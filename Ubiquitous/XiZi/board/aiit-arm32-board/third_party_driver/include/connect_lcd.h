/*
* Copyright (c) 2020 AIIT XUOS Lab
* XiUOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/
 
/**
* @file connect_lcd.h
* @brief declare aiit-arm32-board lcd fsmc function
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-1-12
*/

#ifndef CONNECT_LCD_H
#define CONNECT_LCD_H

#include <device.h>
#include <font.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ILI9341 0x9341

#define NO_OPERATION                  0x00
#define SOFTWARE_RESET                0x01
#define READ_ID                       0x04
#define READ_STATUS                   0x09
#define READ_POWER_MODE               0x0A
#define READ_MADCTL                   0x0B
#define READ_PIXEL_FORMAT             0x0C
#define READ_IMAGE_FORMAT             0x0D
#define READ_SIGNAL_MODE              0x0E
#define READ_SELT_DIAG_RESULT         0x0F
#define SLEEP_ON                      0x10
#define SLEEP_OFF                     0x11
#define PARTIAL_DISPALY_ON            0x12
#define NORMAL_DISPALY_ON             0x13
#define INVERSION_DISPALY_OFF         0x20
#define INVERSION_DISPALY_ON          0x21
#define GAMMA_SET                     0x26
#define DISPALY_OFF                   0x28
#define DISPALY_ON                    0x29
#define HORIZONTAL_ADDRESS_SET        0x2A
#define VERTICAL_ADDRESS_SET          0x2B
#define MEMORY_WRITE                  0x2C
#define COLOR_SET                     0x2D
#define MEMORY_READ                   0x2E
#define PARTIAL_AREA                  0x30
#define VERTICAL_SCROL_DEFINE         0x33
#define TEAR_EFFECT_LINE_OFF          0x34
#define TEAR_EFFECT_LINE_ON           0x35
#define MEMORY_ACCESS_CTL             0x36
#define VERTICAL_SCROL_S_ADD          0x37
#define IDLE_MODE_OFF                 0x38
#define IDLE_MODE_ON                  0x39
#define PIXEL_FORMAT_SET              0x3A
#define WRITE_MEMORY_CONTINUE         0x3C
#define READ_MEMORY_CONTINUE          0x3E
#define SET_TEAR_SCANLINE             0x44
#define GET_SCANLINE                  0x45
#define WRITE_BRIGHTNESS              0x51
#define READ_BRIGHTNESS               0x52
#define WRITE_CTRL_DISPALY            0x53
#define READ_CTRL_DISPALY             0x54
#define WRITE_BRIGHTNESS_CTL          0x55
#define READ_BRIGHTNESS_CTL           0x56
#define WRITE_MIN_BRIGHTNESS          0x5E
#define READ_MIN_BRIGHTNESS           0x5F
#define READ_ID1                      0xDA
#define READ_ID2                      0xDB
#define READ_ID3                      0xDC
#define RGB_IF_SIGNAL_CTL             0xB0
#define NORMAL_FRAME_CTL              0xB1
#define IDLE_FRAME_CTL                0xB2
#define PARTIAL_FRAME_CTL             0xB3
#define INVERSION_CTL                 0xB4
#define BLANK_PORCH_CTL               0xB5
#define DISPALY_FUNCTION_CTL          0xB6
#define ENTRY_MODE_SET                0xB7
#define BACKLIGHT_CTL1                0xB8
#define BACKLIGHT_CTL2                0xB9
#define BACKLIGHT_CTL3                0xBA
#define BACKLIGHT_CTL4                0xBB
#define BACKLIGHT_CTL5                0xBC
#define BACKLIGHT_CTL7                0xBE
#define BACKLIGHT_CTL8                0xBF
#define POWER_CTL1                    0xC0
#define POWER_CTL2                    0xC1
#define VCOM_CTL1                     0xC5
#define VCOM_CTL2                     0xC7
#define NV_MEMORY_WRITE               0xD0
#define NV_MEMORY_PROTECT_KEY         0xD1
#define NV_MEMORY_STATUS_READ         0xD2
#define READ_ID4                      0xD3
#define POSITIVE_GAMMA_CORRECT        0xE0
#define NEGATIVE_GAMMA_CORRECT        0xE1
#define DIGITAL_GAMMA_CTL1            0xE2
#define DIGITAL_GAMMA_CTL2            0xE3
#define INTERFACE_CTL                 0xF6

#define LCD_VERTICAL_SCEEN 0
#define LCD_HORIZONTAL_SCEEN 1

//LCD Scan Direction
#define L2R_U2D  0 //from left to right , from up to down
#define L2R_D2U  1 
#define R2L_U2D  2 
#define R2L_D2U  3 

#define U2D_L2R  4 
#define U2D_R2L  5 
#define D2U_L2R  6 
#define D2U_R2L  7 	 

#define DFT_SCAN_DIR  U2D_R2L  //default scan direction

#define FSMC_BANK1_NORSRAM4_START_ADDRESS 0x6C000000
#define FSMC_BANK1_NORSRAM4_DATA_WIDTH 16
#define STM32_FSMC_OFFSET 0x0000007E

typedef struct
{
	volatile uint16_t LCD_REG_CMD;//0x6C00007E, ADDR_A6 = 0, RS =0, write CMD
	volatile uint16_t LCD_RAM_DATA;//0x6C000080, ADDR_A6 =1, RS = 1, write DATA
} LCD_TypeDef;
		    
#ifndef AIIT_BOARD_LCD_BASE            
#define AIIT_BOARD_LCD_BASE  ((uint32_t)(FSMC_BANK1_NORSRAM4_START_ADDRESS | STM32_FSMC_OFFSET))
#define AIIT_BOARD_LCD       ((LCD_TypeDef *) AIIT_BOARD_LCD_BASE)
#endif

typedef struct 
{ 
    uint16_t width;
    uint16_t height;
    uint16_t lcd_id;
    uint8_t lcd_direction; //0，Vertical screen；1，Horizontal screen
    uint16_t write_ram_cmd;
    uint16_t set_x_cmd;
    uint16_t set_y_cmd;
} LCD_PARAM;

struct Stm32HwLcd
{
    LCD_TypeDef *LCD;
    LCD_PARAM lcd_param;
};

int Stm32HwLcdInit(void);

#ifdef __cplusplus
}
#endif

#endif