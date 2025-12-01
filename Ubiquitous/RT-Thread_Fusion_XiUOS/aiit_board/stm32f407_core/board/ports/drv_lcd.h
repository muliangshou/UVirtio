/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-28     unknow       copy by STemwin
 */
#ifndef __DRV_LCD_H
#define __DRV_LCD_H
#include <rtthread.h>
#include "rtdevice.h"
#include <drv_common.h>
#define LCD_W 800
#define LCD_H 480


//LCD重要参数集
typedef struct
{
    uint16_t width;         //LCD 宽度
    uint16_t height;        //LCD 高度
    uint16_t id;            //LCD ID
    uint8_t  dir;           //横屏还是竖屏控制：0，竖屏；1，横屏。
    uint16_t wramcmd;       //开始写gram指令
    uint16_t setxcmd;       //设置x坐标指令
    uint16_t setycmd;       //设置y坐标指令
    uint16_t pointcolor;     // 画笔颜色
    uint16_t backcolor;      //背景色
}_lcd_dev;

//LCD参数
extern _lcd_dev lcddev; //管理LCD重要参数

typedef struct
{
  __IO uint16_t REG;
  __IO uint16_t RAM;
}LCD_CONTROLLER_TypeDef;

//扫描方向定义
#define L2R_U2D  0      //从左到右,从上到下
#define L2R_D2U  1      //从左到右,从下到上
#define R2L_U2D  2      //从右到左,从上到下
#define R2L_D2U  3      //从右到左,从下到上

#define U2D_L2R  4      //从上到下,从左到右
#define U2D_R2L  5      //从上到下,从右到左
#define D2U_L2R  6      //从下到上,从左到右
#define D2U_R2L  7      //从下到上,从右到左

#define DFT_SCAN_DIR  L2R_U2D  //默认的扫描方向

//LCD分辨率设置
#define SSD_HOR_RESOLUTION      800     //LCD水平分辨率
#define SSD_VER_RESOLUTION      480     //LCD垂直分辨率
//LCD驱动参数设置
#define SSD_HOR_PULSE_WIDTH     1       //水平脉宽
#define SSD_HOR_BACK_PORCH      46      //水平前廊
#define SSD_HOR_FRONT_PORCH     210     //水平后廊

#define SSD_VER_PULSE_WIDTH     1       //垂直脉宽
#define SSD_VER_BACK_PORCH      23      //垂直前廊
#define SSD_VER_FRONT_PORCH     22      //垂直前廊
//如下几个参数，自动计算
#define SSD_HT  (SSD_HOR_RESOLUTION+SSD_HOR_BACK_PORCH+SSD_HOR_FRONT_PORCH)
#define SSD_HPS (SSD_HOR_BACK_PORCH)
#define SSD_VT  (SSD_VER_RESOLUTION+SSD_VER_BACK_PORCH+SSD_VER_FRONT_PORCH)
#define SSD_VPS (SSD_VER_BACK_PORCH)


//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅绿色
//#define LIGHTGRAY        0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)

void lcd_fill_array(rt_uint16_t x_start, rt_uint16_t y_start, rt_uint16_t x_end, rt_uint16_t y_end, void *pcolor);

#endif
