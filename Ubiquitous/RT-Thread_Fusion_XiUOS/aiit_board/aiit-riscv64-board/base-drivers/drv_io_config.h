/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-19     ZYH          first version
 */

#ifndef __DRV_IO_CONFIG_H__
#define __DRV_IO_CONFIG_H__

#include <rtconfig.h>

enum HS_GPIO_CONFIG
{
#ifdef BSP_USING_LCD
    LCD_DC_PIN = 0,     /* LCD DC PIN */
#if BSP_LCD_RST_PIN >= 0
    LCD_RST_PIN,
#endif
#if BSP_LCD_BACKLIGHT_PIN >= 0
    LCD_BACKLIGHT_PIN,
#endif
#endif
#ifdef BSP_SPI1_USING_SS0
    SPI1_CS0_PIN,
#endif
#ifdef BSP_SPI1_USING_SS1
    SPI1_CS1_PIN,
#endif
#ifdef BSP_SPI1_USING_SS2
    SPI1_CS2_PIN,
#endif
#ifdef BSP_SPI1_USING_SS3
    SPI1_CS3_PIN,
#endif

#ifdef BSP_USING_BRIDGE
    SPI2_INT_PIN,
    SPI2_READY_PIN,
#endif
    GPIO_ALLOC_START /* index of gpio driver start */
};

#define FPIOA_CH438_ALE   11
#define FPIOA_CH438_NWR   12
#define FPIOA_CH438_NRD   13
#define FPIOA_CH438_D0    14
#define FPIOA_CH438_D1    15
#define FPIOA_CH438_D2    16
#define FPIOA_CH438_D3    17
#define FPIOA_CH438_D4    18
#define FPIOA_CH438_D5    19
#define FPIOA_CH438_D6    20
#define FPIOA_CH438_D7    21
#define FPIOA_CH438_INT   22
#define FPIOA_485_DIR     23

//PIN.define
#define BSP_CH438_ALE_PIN   23
#define BSP_CH438_NWR_PIN   24
#define BSP_CH438_NRD_PIN   25

#define BSP_CH438_D0_PIN   27
#define BSP_CH438_D1_PIN   28
#define BSP_CH438_D2_PIN   29
#define BSP_CH438_D3_PIN   30
#define BSP_CH438_D4_PIN   31
#define BSP_CH438_D5_PIN   32
#define BSP_CH438_D6_PIN   33
#define BSP_CH438_D7_PIN   34
#define BSP_CH438_INT_PIN  35

extern int io_config_init(void);

#endif
