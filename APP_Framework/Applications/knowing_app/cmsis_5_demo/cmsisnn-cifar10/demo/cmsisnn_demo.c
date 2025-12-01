#include <transform.h>
#include <tjpgd.h>
#include "../model/m4/nn.h"

#define WORK_POOL_SIZE (4 * 1024 + 32)

const char *cifar10_label[] = {"Plane", "Car", "Bird", "Cat", "Deer", "Dog", "Frog", "Horse", "Ship", "Truck"};

int get_top_prediction(q7_t *predictions)
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

int cmsisnn_inference(uint8_t *input_data)
{
  q7_t output_data[10];
  run_nn((q7_t *)input_data, output_data);
  arm_softmax_q7(output_data, IP1_OUT_DIM, output_data);
  int top_ind = get_top_prediction(output_data);
  printf("\rPrediction: %s \r\n", cifar10_label[top_ind]);
  return top_ind;
}

typedef struct
{
  FILE *fp;
  uint8_t *fbuf;
  uint16_t wfbuf;
} IODEV;

unsigned int in_func_cmsisnn(JDEC *jd, uint8_t *buff, unsigned int nbyte)
{
  IODEV *dev = (IODEV *)jd->device;

  if (buff)
  {
    return (uint16_t)fread(buff, 1, nbyte, dev->fp);
  }
  else
  {
    return fseek(dev->fp, nbyte, SEEK_CUR) ? 0 : nbyte;
  }
}

int out_func_cmsisnn(JDEC *jd, void *bitmap, JRECT *rect)
{
  IODEV *dev = (IODEV *)jd->device;
  uint8_t *src, *dst;
  uint16_t y, bws, bwd;

  if (rect->left == 0)
  {
    printf("\r%lu%%", (rect->top << jd->scale) * 100UL / jd->height);
  }

  src = (uint8_t *)bitmap;
  dst = dev->fbuf + 3 * (rect->top * dev->wfbuf + rect->left);
  bws = 3 * (rect->right - rect->left + 1);
  bwd = 3 * dev->wfbuf;
  for (y = rect->top; y <= rect->bottom; y++)
  {
    memcpy(dst, src, bws);
    src += bws;
    dst += bwd;
  }

  return 1;
}

int cmsisnn_demo(int argc, char *argv[])
{
  void *work;
  JDEC jdec; 
  JRESULT res; 
  IODEV devid; 

  if (argc < 2)
  {
    printf("Jpeg_Dec illegal arguments ...\n");
    return -1;
  }

  devid.fp = fopen(argv[1], "r+");
  if (!devid.fp)
  {
    printf("Jpeg_Dec open the file failed...\n");
    return -1;
  }

  work = malloc(WORK_POOL_SIZE);
  if (work == NULL)
  {
    printf("Jpeg_Dec work malloc failed...\n");
    res = -1;
    goto __exit;
  }

  res = jd_prepare(&jdec, in_func_cmsisnn, work, WORK_POOL_SIZE, &devid);
  if (res == JDR_OK)
  {
    printf("Image dimensions: %u by %u. %u bytes used.\n", jdec.width, jdec.height, 3100 - jdec.sz_pool);

    devid.fbuf = malloc(3 * jdec.width * jdec.height);
    if (devid.fbuf == NULL)
    {
      printf("Jpeg_Dec devid.fbuf malloc failed, need to use %d Bytes ...\n", 3 * jdec.width * jdec.height);
      res = -1;
      goto __exit;
    }
    devid.wfbuf = jdec.width;

    res = jd_decomp(&jdec, out_func_cmsisnn, 0);
    if (res == JDR_OK)
    {
      printf("\rDecompress success \n");
    }
    else
    {
      printf("Failed to decompress: rc=%d\n", res);
    }

    cmsisnn_inference(devid.fbuf);

    if (devid.fbuf != NULL)
    {
      free(devid.fbuf);
    }
  }
  else
  {
    printf("Failed to prepare: rc=%d\n", res);
  }

__exit:
  if (work != NULL)
  {
    free(work);
  }

  fclose(devid.fp);

  return res;
}

#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(cmsisnn_demo, cifar10 demo and filename should be followed);
#endif