#include <rtthread.h>
#include <rtdevice.h>
#include "stdio.h"
#include "string.h"

#ifdef OV2640_RGB565_MODE
#ifdef RT_USING_POSIX
#include <dfs_posix.h>
#include <dfs_poll.h>
#ifdef RT_USING_POSIX_TERMIOS
#include <posix_termios.h>
#endif
#endif

#include <drv_ov2640.h>
#include <drv_lcd.h>
#include "nn_vegetable_classify.h"
#define JPEG_BUF_SIZE (2 * OV2640_X_RESOLUTION_IMAGE_OUTSIZE * OV2640_Y_RESOLUTION_IMAGE_OUTSIZE)
#define IOCTL_ERROR 1

static int fd = 0;
static int infer_times = 0;
static int photo_times = 0;
static int height = OV2640_X_RESOLUTION_IMAGE_OUTSIZE;
static int width = OV2640_Y_RESOLUTION_IMAGE_OUTSIZE;
static _ioctl_shoot_para shoot_para_t = {0};
const char *vegetable_label[] = {"mushroom", "pepper", "potato", "tomato"};

uint8_t *resized_buffer = NULL;
uint8_t *in_buffer = NULL;

int get_top_prediction_detection(q7_t *predictions)
{
  int max_ind = 0;
  int max_val = -128;
  for (int i = 0; i < 10; i++)
  {
    if (max_val < predictions[i])
    {
      max_val = predictions[i];
      max_ind = i;
    }
  }
  return max_ind;
}

int cmsisnn_inference_vegetable_classify(uint8_t *input_data)
{
  int8_t output_data[4];
  char output[50] = {0};
  char outputPrediction[50] = {0};
  memset(output, 0, 50);
  memset(outputPrediction, 0, 50);

  run_nn_sn_classify((int8_t *)input_data, output_data);
  arm_softmax_q7(output_data, 4, output_data);

  infer_times++;
  int top_ind = get_top_prediction_detection(output_data);
  printf("times:%d  Prediction:%s \r\n", infer_times, vegetable_label[top_ind]);
  sprintf(outputPrediction, "times:%d Prediction:%s        \r\n", infer_times, vegetable_label[top_ind]);
  lcd_show_string(1, 280, 240, 16, 16, outputPrediction, RED);
  return top_ind;
}

void resize_rgb888in_rgb565out(uint8_t *camera_image, uint16_t *resize_image)
{
  uint8_t *psrc_temp = (uint8_t *)camera_image;
  uint16_t *pdst_temp = (uint16_t *)resize_image;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *pdst_temp++ = (*psrc_temp++ & 0xF8) << 8 | (*psrc_temp++ & 0xFC) << 3 | *psrc_temp++ >> 3;
    }
  }
}

void resize_rgb565in_rgb888out(uint8_t *camera_image, uint8_t *resize_image)
{
  uint8_t *psrc_temp = (uint8_t *)camera_image;
  uint8_t *pdst_temp = (uint8_t *)resize_image;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      uint8_t pixel_lo = *psrc_temp++;
      uint8_t pixel_hi = *psrc_temp++;
      *pdst_temp++ = (0xF8 & pixel_hi);
      *pdst_temp++ = ((0x07 & pixel_hi) << 5) | ((0xE0 & pixel_lo) >> 3);
      *pdst_temp++ = (0x1F & pixel_lo) << 3;
    }
  }
}

void lcd_show_ov2640_thread_detection(uint8_t *rgbbuffer)
{
  int32_t ret = 0;
  while (1)
  {
    ret = ioctl(fd, IOCTRL_CAMERA_START_SHOT, &shoot_para_t);
    if (ret == IOCTL_ERROR)
    {
      printf("ov2640 can't wait event flag");
      free(rgbbuffer);
      return;
    }
    lcd_fill_array(0, 0, OV2640_X_RESOLUTION_IMAGE_OUTSIZE, OV2640_Y_RESOLUTION_IMAGE_OUTSIZE, rgbbuffer);

    if (photo_times % 20 == 0)
    {
      resize_rgb565in_rgb888out(rgbbuffer, resized_buffer);
      int pixel = 0;
      for (int i = 0; i < 3 * width; i += 3 * width / CONV1_IN_DIM)
      {
        for (int j = 0; j < 3 * height; j += 3 * height / CONV1_IN_DIM)
        {
          for (int k = 0; k < 3; k++, pixel++)
          {
            *(in_buffer + pixel) = *(resized_buffer + 256 * i + j + k);
          }
        }
      }
      cmsisnn_inference_vegetable_classify(in_buffer);
    }
    photo_times++;
  }
}

void cmsisnn_vegetable_classify()
{
  fd = open("/dev/ov2640", O_RDONLY);
  if (fd < 0)
  {
    printf("open ov2640 fail !!");
    return;
  }
  printf("memory_init \n\r");
  uint8_t *JpegBuffer = malloc(JPEG_BUF_SIZE);
  if (JpegBuffer == NULL)
  {
    printf("JpegBuffer senddata buf malloc error!\n");
    return;
  }
  resized_buffer = malloc(3 * width * height);
  if (resized_buffer == NULL)
  {
    printf("Resized_buffer buf malloc error!\n");
    return;
  }
  in_buffer = malloc(CONV1_IN_CH * CONV1_IN_DIM * CONV1_IN_DIM);
  if (in_buffer == NULL)
  {
    printf("In_buffer buf malloc error!\n");
    return;
  }
  memory_init();
  printf("memory_init success\n\r");
  
  shoot_para_t.pdata = (uint32_t)JpegBuffer;
  shoot_para_t.length = JPEG_BUF_SIZE / 2;

  int result = 0;
  pthread_t tid = 0;
  pthread_attr_t attr;
  struct sched_param prio;
  prio.sched_priority = 8;
  size_t stack_size = 1024 * 11;
  pthread_attr_init(&attr);
  pthread_attr_setschedparam(&attr, &prio); 
  pthread_attr_setstacksize(&attr, stack_size);

  result = pthread_create(&tid, &attr, lcd_show_ov2640_thread_detection, JpegBuffer);
  if (0 == result) {
      printf("thread_detect_entry successfully!\n");
  } else {
      printf("thread_detect_entry failed! error code is %d.\n", result);
      close(fd);
  }

  return;
}

#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(cmsisnn_vegetable_classify, classify vegetable using cmsis-nn);
#endif

#endif