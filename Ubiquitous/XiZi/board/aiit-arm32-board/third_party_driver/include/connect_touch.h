/*
 * Copyright (c) Guangzhou Xingyi Electronic  Technology Co., Ltd
 *
 * Change Logs:
 * Date                     Author              Notes
 * 2014-5-7        alientek team   first version
 */

/**
* @file connect_touch.c
* @brief support aiit-arm32-board touch function and register to bus framework
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-04-25
*/
#ifndef CONNECT_TOUCH_H
#define CONNECT_TOUCH_H

#include <device.h>
#include <stm32f4xx.h>

#define TP_PRES_DOWN 0x80  
#define TP_CATH_PRES   0x40  

//touch screen control struct
typedef struct
{
	u16 x; 	
	u16 y;					
	u8  sta;								
	float xfac;					
	float yfac;
	short xoff;
	short yoff;	   
	u8 touchtype;
}touch_device_info;

extern touch_device_info tp_dev;	 	

//save data struct
typedef struct 
{
	s32  ty_xfac;
	s32  ty_yfac;
	short  x_pos;
	short  y_pos;
	u8  iic_touchtype;        
	u8  iic_flag;              
}TP_modify_save;

//io pin define
#define PEN  	    	          PDin(6)  	
#define  T_MISO	              PBin(4)   	
#define T_MOSI 	        	  PBout(5)  
#define T_CLK 	         	    PBout(3)  	
#define TCS  	 	               PGout(13)  	
   
int Stm32HwTouchBusInit(void);

#endif
