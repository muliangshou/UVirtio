#include <rtthread.h>
#include <rtdevice.h>
#include "stdio.h"
#include "string.h"
#ifdef DRV_USING_OV2640
#ifdef RT_USING_POSIX
#include <dfs_posix.h>
#include <dfs_poll.h>
#ifdef RT_USING_POSIX_TERMIOS
#include <posix_termios.h>
#endif
#endif
#include<drv_ov2640.h>
#ifdef OV2640_JPEG_MODE
#define JPEG_BUF_SIZE (1024*200)
#else
#define JPEG_BUF_SIZE (2*OV2640_X_RESOLUTION_IMAGE_OUTSIZE*OV2640_Y_RESOLUTION_IMAGE_OUTSIZE)
#endif
#define UART_NUMBER2                 "uart2"
void lcd_show_ov2640_thread(uint16_t* rgbbuffer);
static int fd = 0;
static  _ioctl_shoot_para shoot_para_t = {0};
void ov2640_test(int argc, char **argv)
{
    rt_thread_t tid;
    rt_err_t ret = 0;
    fd = open("/dev/ov2640",O_RDONLY);
    if(fd < 0)
    {
        printf("open ov2640 fail !!");
        return;
    }
    rt_uint8_t* JpegBuffer = rt_malloc(JPEG_BUF_SIZE);
    if (RT_NULL == JpegBuffer)
   {
       printf("JpegBuffer senddata buf malloc error!\n");
       return;
   }
    printf("ov2640 test by printing the image value in memory \r\n");
    shoot_para_t.pdata = (uint32_t)JpegBuffer;
    #ifdef OV2640_RGB565_MODE
    shoot_para_t.length = JPEG_BUF_SIZE/2;
    #elif defined OV2640_JPEG_MODE
    shoot_para_t.length = JPEG_BUF_SIZE;
    #endif
    ret = ioctl(fd,IOCTRL_CAMERA_START_SHOT,&shoot_para_t);
    if(RT_ERROR == ret)
    {
        printf("ov2640 can't wait event flag");
        return;
    }
    #ifdef OV2640_JPEG_MODE
        printf("print the vaule:\r\n\r\n");        
        ret = rt_ov2640_calculate_jpeg_len(JpegBuffer,JPEG_BUF_SIZE);
        printf("photo leghth is %d :\r\n\r\n",ret);
            #ifdef BSP_USING_UART2
            void img_output_uart2(rt_uint8_t* jpegbuf,rt_uint16_t len);
            img_output_uart2(JpegBuffer,ret);
            #endif
        for(int i =0;i<ret;i++)
        {
            printf("%x",*(JpegBuffer+i));
        }
        printf("\r\n\r\n above :\r\n\r\n");
        rt_free(JpegBuffer);
        close(fd);
    #elif defined OV2640_RGB565_MODE
        tid = rt_thread_create("lcdshow", lcd_show_ov2640_thread, JpegBuffer,3000, 9, 20);
        rt_thread_startup(tid);
    #endif
    return;
}
MSH_CMD_EXPORT(ov2640_test,printing the image value in memory);

#ifdef BSP_USING_UART2
void img_output_uart2(rt_uint8_t* jpegbuf,rt_uint16_t len)
{
    static rt_uint8_t i =0;
    static rt_device_t serial = NULL;
    int ret = RT_EOK;
    if(i == 0)
    {
        i = 1;
        serial = rt_device_find(UART_NUMBER2);
        if (!serial)
        {
            printf("find uart2 failed!\n");
            i = 2;
            return;    
        }
        ret = rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        if(ret != RT_EOK)
        {
             printf("open uart2 failed!\n");
             i = 2;
             return ;
        }
    }
    if(i == 1)
    {
         rt_device_write(serial, 0, jpegbuf, len);
    }
    
}
#endif
#ifdef OV2640_RGB565_MODE
extern void lcd_fill_array(rt_uint16_t x_start, rt_uint16_t y_start, rt_uint16_t x_end, rt_uint16_t y_end, void *pcolor);
void lcd_show_ov2640_thread(uint16_t* rgbbuffer)
{
    rt_err_t ret = 0;
    while(1)
    {
    
        ret = ioctl(fd,IOCTRL_CAMERA_START_SHOT,&shoot_para_t);
        if(RT_ERROR == ret)
        {
            printf("ov2640 can't wait event flag");
            rt_free(rgbbuffer);
            return;
        }
        //lcd_show_image(0, 0, 320, 240, rgbbuffer); 
        lcd_fill_array(0,0,OV2640_X_RESOLUTION_IMAGE_OUTSIZE,OV2640_Y_RESOLUTION_IMAGE_OUTSIZE,rgbbuffer);
        rt_thread_mdelay(1);
    }
    
}
#endif

#endif

