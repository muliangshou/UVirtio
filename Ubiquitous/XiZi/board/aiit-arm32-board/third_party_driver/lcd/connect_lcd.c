/*
 * Copyright (c) Guangzhou Xingyi Electronic Technology Co., Ltd
 *
 * Change Logs:
 * Date          Author     Notes
 * 2014-7-4      alientek   first version
 */

/**
* @file connect_lcd.c
* @brief support aiit-arm32-board lcd function and register to bus framework
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-1-12
*/

/*************************************************
File name: connect_lcd.c
Description: support aiit-arm32-board lcd configure and lcd bus register function
Others: take hardware/lcd/lcd.c for references
History: 
1. Date: 2022-1-12
Author: AIIT XUOS Lab
Modification: 
1. support aiit-arm32-board lcd configure, write by FSMC function
2. support aiit-arm32-board lcd bus device and driver register
*************************************************/

#include <connect_lcd.h>
#include <connect_gpio.h>
#include <hardware_fsmc.h>
#include <hardware_gpio.h>
#include <hardware_rcc.h>

static int Stm32LcdUdelay(uint32 us)
{
    uint32 ticks;
    uint32 told, tnow, tcnt = 0;
    uint32 reload = SysTick->LOAD;

    ticks = us * reload / (1000000 / TICK_PER_SECOND);
    told = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                return 0;
                break;
            }
        }
    }
}

static void HwFsmcInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef readWriteTiming; 
    FSMC_NORSRAMTimingInitTypeDef writeTiming;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG, ENABLE); 
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PD3, LCD back light
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//PF12, FSMC_A6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//PG12, FSMC_NE4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);//PD0,AF12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);//PD1,AF12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);//PD4,AF12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);//PD5,AF12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);//PD8,AF12 
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);//PD9,AF12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);//PD10,AF12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);//PD14,AF12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);//PD15,AF12

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);//PE7,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);//PE8,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);//PE9,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);//PE10,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);//PE11,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);//PE12,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);//PE13,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);//PE14,AF12
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);//PE15,AF12

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);//PF12,AF12
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);//PG12,AF12

    readWriteTiming.FSMC_AddressSetupTime = 0xF;//16 HCLK 1/168M=6ns*16=96ns	
    readWriteTiming.FSMC_AddressHoldTime = 0x00;	
    readWriteTiming.FSMC_DataSetupTime = 60;//60 HCLK 6*60=360ns
    readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
    readWriteTiming.FSMC_CLKDivision = 0x00;
    readWriteTiming.FSMC_DataLatency = 0x00;
    readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;//mode A

    writeTiming.FSMC_AddressSetupTime = 9;//9 HCLK =54ns 
    writeTiming.FSMC_AddressHoldTime = 0x00;		
    writeTiming.FSMC_DataSetupTime = 8;//9 HCLK=54ns
    writeTiming.FSMC_BusTurnAroundDuration = 0x00;
    writeTiming.FSMC_CLKDivision = 0x00;
    writeTiming.FSMC_DataLatency = 0x00;
    writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;//mode A 

    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;//FSMC_NE4, register BTCR[6],[7]。
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;   
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//data bit width 16bit   
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &writeTiming;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);//init FSMC init

    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);//enable BANK1 
}

static void LcdWriteCmd(LCD_TypeDef *LCD, uint16_t cmd)
{
	LCD->LCD_REG_CMD = cmd;
}

static void LcdWriteData(LCD_TypeDef *LCD, uint16_t data)
{
	LCD->LCD_RAM_DATA = data;
}

static void LcdWriteReg(LCD_TypeDef *LCD, uint16_t cmd, uint16_t data)
{
	LCD->LCD_REG_CMD = cmd;
    LCD->LCD_RAM_DATA = data;
}

static uint16_t LcdReadData(LCD_TypeDef *LCD)
{
	uint16_t data;
    data = LCD->LCD_RAM_DATA;
    return data;
}

static uint16_t LcdReadReg(LCD_TypeDef *LCD, uint16_t cmd)
{
	LcdWriteCmd(LCD, cmd);
    Stm32LcdUdelay(1000);
    return LcdReadData(LCD);
}

static void LcdDisplayOn(LCD_TypeDef *LCD)
{
    LcdWriteCmd(LCD, DISPALY_ON);
}

static void LcdDisplayOff(LCD_TypeDef *LCD)
{
    LcdWriteCmd(LCD, DISPALY_OFF);
}

static void LcdWriteRamCmd(struct Stm32HwLcd *hw_lcd)
{
 	LcdWriteCmd(hw_lcd->LCD, hw_lcd->lcd_param.write_ram_cmd);	  
}	 

static void LcdWriteRamColor(struct Stm32HwLcd *hw_lcd, uint16_t color)
{
 	LcdWriteData(hw_lcd->LCD, color);	  
}

static void LcdSetCursor(struct Stm32HwLcd *hw_lcd, uint16_t x_pos, uint16_t y_pos)
{
    LcdWriteCmd(hw_lcd->LCD, hw_lcd->lcd_param.set_x_cmd);
    LcdWriteData(hw_lcd->LCD, x_pos >> 8);
    LcdWriteData(hw_lcd->LCD, x_pos & 0xFF);
    LcdWriteCmd(hw_lcd->LCD, hw_lcd->lcd_param.set_y_cmd);
    LcdWriteData(hw_lcd->LCD, y_pos >> 8);
    LcdWriteData(hw_lcd->LCD, y_pos & 0xFF);
}

static void LcdDrawPoint(struct Stm32HwLcd *hw_lcd, uint16_t x, uint16_t y, uint16_t color)
{
    LcdSetCursor(hw_lcd, x, y);
    LcdWriteRamCmd(hw_lcd);
    LcdWriteRamColor(hw_lcd, color);
}

//(start_x,start_y),(end_x,end_y), area: (end_x-start_x+1)*(end_y-start_y+1)
static void LcdDrawArea(struct Stm32HwLcd *hw_lcd, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t *color)
{
    uint16_t height, width;
    uint16_t i, j;
    width = end_x - start_x + 1;
    height = end_y - start_y + 1;

    for (i = 0; i < height; i++) {
        LcdSetCursor(hw_lcd, start_x, start_y + i);
        LcdWriteRamCmd(hw_lcd);

        for (j = 0; j < width; j++) {
            LcdWriteRamColor(hw_lcd, color[i * width + j]);
        }
    }
}

//mode 0 : draw char, default configure ; 1 : draw char over background
static void LcdDrawChar(struct Stm32HwLcd *hw_lcd, uint16_t x, uint16_t y, uint8_t char_data, uint8_t size, uint8_t mode, uint16_t color,uint16_t back_color)
{  							  
    uint8_t temp, t1, t;
	uint16_t y0 = y;
	uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);	
 	char_data = char_data - ' ';
	for(t = 0;t < csize;t ++) {   
		if(12 == size)
            temp = asc2_1206[char_data][t];
		else if(16 == size)
            temp = asc2_1608[char_data][t];
		else if(24 == size)
            temp = asc2_2412[char_data][t];
		else 
            return;

		for(t1 = 0;t1 < 8;t1 ++) {			    
			if(temp & 0x80)
                LcdDrawPoint(hw_lcd, x, y, color);
			else if(mode == 0)
                LcdDrawPoint(hw_lcd, x, y, back_color);

			temp <<= 1;
			y++;

			if(y >= hw_lcd->lcd_param.height)
                return;//over height

			if((y - y0) == size) {
				y = y0;
				x++;
				if(x >= hw_lcd->lcd_param.width)
                    return;//over width
				break;
			}
		}  	 
	}  	    	   	 	  
}  

static void LcdDrawString(struct Stm32HwLcd *hw_lcd, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p, uint16_t color, uint16_t back_color)
{
    uint8_t x0 = x;
    width += x;
    height += y;

    while ((*p <= '~') && (*p >= ' ')) {
        if (x >= width) {
            x = x0;
            y += size;
        }

        if (y >= height)
            break;

        LcdDrawChar(hw_lcd, x, y, *p, size, 0, color, back_color);
        x += size / 2;
        p++;
    }  
}

static void LcdClear(struct Stm32HwLcd *hw_lcd, uint16_t color)
{
    uint32_t dot_index = 0;
    uint32_t totalpoint = hw_lcd->lcd_param.width * hw_lcd->lcd_param.height;

    LcdSetCursor(hw_lcd, 0x00, 0x00);
    LcdWriteRamCmd(hw_lcd);

    for (dot_index = 0; dot_index < totalpoint; dot_index ++) {
        LcdWriteRamColor(hw_lcd, color);
    }
}

static void LcdScanDirection(struct Stm32HwLcd *hw_lcd, uint8_t scan_direction)
{
    uint16_t cmd = 0;
    uint16_t temp;

    switch (scan_direction) {
        case L2R_U2D:
            cmd |= (0 << 7) | (0 << 6) | (0 << 5);
            break;
        case L2R_D2U:
            cmd |= (1 << 7) | (0 << 6) | (0 << 5);
            break;
        case R2L_U2D:
            cmd |= (0 << 7) | (1 << 6) | (0 << 5);
            break;
        case R2L_D2U:
            cmd |= (1 << 7) | (1 << 6) | (0 << 5);
            break;
        case U2D_L2R:
            cmd |= (0 << 7) | (0 << 6) | (1 << 5);
            break;
        case U2D_R2L:
            cmd |= (0 << 7) | (1 << 6) | (1 << 5);
            break;
        case D2U_L2R:
            cmd |= (1 << 7) | (0 << 6) | (1 << 5);
            break;
        case D2U_R2L:
            cmd |= (1 << 7) | (1 << 6) | (1 << 5);
            break;
    }

    cmd |= 0X08;//set BGR bit

    LcdWriteReg(hw_lcd->LCD, MEMORY_ACCESS_CTL, cmd);

    if (cmd & INVERSION_DISPALY_OFF) {//reverse x、y
        if (hw_lcd->lcd_param.width < hw_lcd->lcd_param.height) {
                temp = hw_lcd->lcd_param.width;
                hw_lcd->lcd_param.width = hw_lcd->lcd_param.height;
                hw_lcd->lcd_param.height = temp;
        }
    } else {
        if (hw_lcd->lcd_param.width > hw_lcd->lcd_param.height) {
            temp = hw_lcd->lcd_param.width;
            hw_lcd->lcd_param.width = hw_lcd->lcd_param.height;
            hw_lcd->lcd_param.height = temp;
        }
    }

    LcdWriteCmd(hw_lcd->LCD, hw_lcd->lcd_param.set_x_cmd);
    LcdWriteData(hw_lcd->LCD, 0);
    LcdWriteData(hw_lcd->LCD, 0);
    LcdWriteData(hw_lcd->LCD, (hw_lcd->lcd_param.width - 1) >> 8);
    LcdWriteData(hw_lcd->LCD, (hw_lcd->lcd_param.width - 1) & 0xFF);
    LcdWriteCmd(hw_lcd->LCD, hw_lcd->lcd_param.set_y_cmd);
    LcdWriteData(hw_lcd->LCD, 0);
    LcdWriteData(hw_lcd->LCD, 0);
    LcdWriteData(hw_lcd->LCD, (hw_lcd->lcd_param.height - 1) >> 8);
    LcdWriteData(hw_lcd->LCD, (hw_lcd->lcd_param.height - 1) & 0xFF);
}

static void LcdDisplayDirection(struct Stm32HwLcd *hw_lcd)
{
    hw_lcd->lcd_param.write_ram_cmd = MEMORY_WRITE;
    hw_lcd->lcd_param.set_x_cmd = HORIZONTAL_ADDRESS_SET;
    hw_lcd->lcd_param.set_y_cmd = VERTICAL_ADDRESS_SET;

    LcdScanDirection(hw_lcd, DFT_SCAN_DIR);
}

static void HwLcdInit(struct Stm32HwLcd *hw_lcd)
{
    LcdWriteCmd(hw_lcd->LCD, READ_ID4);				   
	hw_lcd->lcd_param.lcd_id = LcdReadData(hw_lcd->LCD);//dummy read 	
	hw_lcd->lcd_param.lcd_id = LcdReadData(hw_lcd->LCD);//0x00
	hw_lcd->lcd_param.lcd_id = LcdReadData(hw_lcd->LCD);//93								   
	hw_lcd->lcd_param.lcd_id <<= 8;
	hw_lcd->lcd_param.lcd_id |= LcdReadData(hw_lcd->LCD);//41 

    KPrintf("HwLcdInit lcd id 0x%x\n", hw_lcd->lcd_param.lcd_id);

    if (ILI9341 == hw_lcd->lcd_param.lcd_id) {
        FSMC_Bank1E->BWTR[6] &= ~(0xF << 0);
		FSMC_Bank1E->BWTR[6] &= ~(0xF << 8);
		FSMC_Bank1E->BWTR[6] |= 3 << 0;	 
        FSMC_Bank1E->BWTR[6] |= 2 << 8;

		LcdWriteCmd(hw_lcd->LCD, 0xCF);  
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteData(hw_lcd->LCD, 0xC1); 
		LcdWriteData(hw_lcd->LCD, 0x30); 
		LcdWriteCmd(hw_lcd->LCD, 0xED);  
		LcdWriteData(hw_lcd->LCD, 0x64); 
		LcdWriteData(hw_lcd->LCD, 0x03); 
		LcdWriteData(hw_lcd->LCD, 0X12); 
		LcdWriteData(hw_lcd->LCD, 0X81); 
		LcdWriteCmd(hw_lcd->LCD, 0xE8);  
		LcdWriteData(hw_lcd->LCD, 0x85); 
		LcdWriteData(hw_lcd->LCD, 0x10); 
		LcdWriteData(hw_lcd->LCD, 0x7A); 
		LcdWriteCmd(hw_lcd->LCD, 0xCB);  
		LcdWriteData(hw_lcd->LCD, 0x39); 
		LcdWriteData(hw_lcd->LCD, 0x2C); 
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteData(hw_lcd->LCD, 0x34); 
		LcdWriteData(hw_lcd->LCD, 0x02); 
		LcdWriteCmd(hw_lcd->LCD, 0xF7);  
		LcdWriteData(hw_lcd->LCD, 0x20); 
		LcdWriteCmd(hw_lcd->LCD, 0xEA);  
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteCmd(hw_lcd->LCD, POWER_CTL1);//Power control 
		LcdWriteData(hw_lcd->LCD, 0x1B);//VRH[5:0] 
		LcdWriteCmd(hw_lcd->LCD, POWER_CTL2);//Power control 
		LcdWriteData(hw_lcd->LCD, 0x01);//SAP[2:0];BT[3:0] 
		LcdWriteCmd(hw_lcd->LCD, VCOM_CTL1);//VCM control 
		LcdWriteData(hw_lcd->LCD, 0x30);
		LcdWriteData(hw_lcd->LCD, 0x30);
		LcdWriteCmd(hw_lcd->LCD, VCOM_CTL2);//VCM control2 
		LcdWriteData(hw_lcd->LCD, 0xB7); 
		LcdWriteCmd(hw_lcd->LCD, MEMORY_ACCESS_CTL);// Memory Access Control 
		LcdWriteData(hw_lcd->LCD, 0x48); 
		LcdWriteCmd(hw_lcd->LCD, PIXEL_FORMAT_SET);   
		LcdWriteData(hw_lcd->LCD, 0x55); 
		LcdWriteCmd(hw_lcd->LCD, NORMAL_FRAME_CTL);   
		LcdWriteData(hw_lcd->LCD, 0x00);   
		LcdWriteData(hw_lcd->LCD, 0x1A); 
		LcdWriteCmd(hw_lcd->LCD, DISPALY_FUNCTION_CTL);// Display Function Control 
		LcdWriteData(hw_lcd->LCD, 0x0A); 
		LcdWriteData(hw_lcd->LCD, 0xA2); 
		LcdWriteCmd(hw_lcd->LCD, 0xF2);// 3Gamma Function Disable 
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteCmd(hw_lcd->LCD, GAMMA_SET);//Gamma curve selected 
		LcdWriteData(hw_lcd->LCD, 0x01); 
		LcdWriteCmd(hw_lcd->LCD, POSITIVE_GAMMA_CORRECT);//Set Gamma 
		LcdWriteData(hw_lcd->LCD, 0x0F); 
		LcdWriteData(hw_lcd->LCD, 0x2A); 
		LcdWriteData(hw_lcd->LCD, 0x28); 
		LcdWriteData(hw_lcd->LCD, 0x08); 
		LcdWriteData(hw_lcd->LCD, 0x0E); 
		LcdWriteData(hw_lcd->LCD, 0x08); 
		LcdWriteData(hw_lcd->LCD, 0x54); 
		LcdWriteData(hw_lcd->LCD, 0xA9); 
		LcdWriteData(hw_lcd->LCD, 0x43); 
		LcdWriteData(hw_lcd->LCD, 0x0A); 
		LcdWriteData(hw_lcd->LCD, 0x0F); 
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteData(hw_lcd->LCD, 0x00); 		 
		LcdWriteCmd(hw_lcd->LCD, NEGATIVE_GAMMA_CORRECT);//Set Gamma 
		LcdWriteData(hw_lcd->LCD, 0x00); 
		LcdWriteData(hw_lcd->LCD, 0x15); 
		LcdWriteData(hw_lcd->LCD, 0x17); 
		LcdWriteData(hw_lcd->LCD, 0x07); 
		LcdWriteData(hw_lcd->LCD, 0x11); 
		LcdWriteData(hw_lcd->LCD, 0x06); 
		LcdWriteData(hw_lcd->LCD, 0x2B); 
		LcdWriteData(hw_lcd->LCD, 0x56); 
		LcdWriteData(hw_lcd->LCD, 0x3C); 
		LcdWriteData(hw_lcd->LCD, 0x05); 
		LcdWriteData(hw_lcd->LCD, 0x10); 
		LcdWriteData(hw_lcd->LCD, 0x0F); 
		LcdWriteData(hw_lcd->LCD, 0x3F); 
		LcdWriteData(hw_lcd->LCD, 0x3F); 
		LcdWriteData(hw_lcd->LCD, 0x0F); 
		LcdWriteCmd(hw_lcd->LCD, VERTICAL_ADDRESS_SET); 
		LcdWriteData(hw_lcd->LCD, 0x00);
		LcdWriteData(hw_lcd->LCD, 0x00);
		LcdWriteData(hw_lcd->LCD, 0x01);
		LcdWriteData(hw_lcd->LCD, 0x3f);
		LcdWriteCmd(hw_lcd->LCD, HORIZONTAL_ADDRESS_SET); 
		LcdWriteData(hw_lcd->LCD, 0x00);
		LcdWriteData(hw_lcd->LCD, 0x00);
		LcdWriteData(hw_lcd->LCD, 0x00);
		LcdWriteData(hw_lcd->LCD, 0xef);	 
		LcdWriteCmd(hw_lcd->LCD, SLEEP_OFF);//Exit Sleep
		Stm32LcdUdelay(120000);
		LcdWriteCmd(hw_lcd->LCD, DISPALY_ON);//display on	

        LcdDisplayDirection(hw_lcd);
        PDout(3) = 1;
        LcdClear(hw_lcd, WHITE);
    }
}

static uint32 LcdWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    x_err_t ret = EOK;

    struct LcdHardwareDevice *lcd_dev = (struct LcdHardwareDevice *)dev;
    struct Stm32HwLcd *hw_lcd = (struct Stm32HwLcd *)lcd_dev->haldev.private_data;

    if (write_param->buffer) {
        LcdWriteParam *show = (LcdWriteParam *)write_param->buffer;
    
        //output string
        if(0 == show->type) {
            LcdDrawString(hw_lcd, show->string_info.x_pos, show->string_info.y_pos, show->string_info.width, show->string_info.height,
                show->string_info.font_size, show->string_info.addr, show->string_info.font_color, show->string_info.back_color);  
        } 
        //output dot
        else if (1 == show->type) {
            LcdDrawArea(hw_lcd, show->pixel_info.x_startpos, show->pixel_info.y_startpos, show->pixel_info.x_endpos, show->pixel_info.y_endpos,
                (uint16_t *)show->pixel_info.pixel_color);
        } else {
            KPrintf("LcdWrite donnot support show type(0 string; 1 dot) %u\n", show->type);
            ret = ERROR;
        }
    }

    return ret;
}

static uint32 LcdControl(void* drv, struct BusConfigureInfo *configure_info)
{
    x_err_t ret = EOK;

    return ret;
}

static const struct LcdDevDone dev_done  = 
{
    NONE,
    NONE,
    LcdWrite,
    NONE,
};

static int BoardLcdBusInit(struct LcdBus * lcd_bus, struct LcdDriver * lcd_driver,const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    /*Init the lcd bus */
    ret = LcdBusInit( lcd_bus, bus_name);
    if (EOK != ret) {
        KPrintf("Board_lcd_init LcdBusInit error %d\n", ret);
        return -ERROR;
    }

    /*Init the lcd driver*/
    ret = LcdDriverInit( lcd_driver, drv_name);
    if (EOK != ret) {
        KPrintf("Board_LCD_init LcdDriverInit error %d\n", ret);
        return -ERROR;
    }

    /*Attach the lcd driver to the lcd bus*/
    ret = LcdDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("Board_LCD_init LcdDriverAttachToBus error %d\n", ret);
        return -ERROR;
    } 

    return ret;
}

/*Attach the lcd device to the lcd bus*/
static int BoardLcdDevBend(struct LcdHardwareDevice *lcd_device, void *param, const char *bus_name, const char *dev_name)
{
    x_err_t ret = EOK;

    ret = LcdDeviceRegister(lcd_device, param, dev_name);
    if (EOK != ret) {
        KPrintf("Board_LCD_init LcdDeviceInit device %s error %d\n", dev_name, ret);
        return -ERROR;
    }  

    ret = LcdDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("Board_LCD_init LcdDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return -ERROR;
    }  

    return  ret;
}

int Stm32HwLcdInit(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_LCD
    static struct LcdBus lcd_bus;
    static struct LcdDriver lcd_drv;
    static struct LcdHardwareDevice lcd_dev;
    static struct Stm32HwLcd hw_lcd;

    memset(&lcd_bus, 0, sizeof(struct LcdBus));
    memset(&lcd_drv, 0, sizeof(struct LcdDriver));
    memset(&lcd_dev, 0, sizeof(struct LcdHardwareDevice));
    memset(&hw_lcd, 0, sizeof(struct Stm32HwLcd));

    lcd_drv.configure = LcdControl;

    ret = BoardLcdBusInit(&lcd_bus, &lcd_drv, LCD_BUS_NAME, LCD_DRV_NAME);
    if (EOK != ret) {
        KPrintf("HwLcdInit BoardLcdBusInit error ret %u\n", ret);
        return ERROR;
    }

    lcd_dev.dev_done = &dev_done;
    hw_lcd.LCD = AIIT_BOARD_LCD;

    /*depends on the LCD hardware*/
    hw_lcd.lcd_param.lcd_direction = LCD_HORIZONTAL_SCEEN;//Horizontal screen
    if (LCD_HORIZONTAL_SCEEN == hw_lcd.lcd_param.lcd_direction) {
        hw_lcd.lcd_param.width = 320;
        hw_lcd.lcd_param.height = 240;
    } else if (LCD_VERTICAL_SCEEN == hw_lcd.lcd_param.lcd_direction) {
        hw_lcd.lcd_param.width = 240;
        hw_lcd.lcd_param.height = 320;
    } else {
        hw_lcd.lcd_param.width = 320;
        hw_lcd.lcd_param.height = 240;
    }

    ret = BoardLcdDevBend(&lcd_dev, (void *)&hw_lcd, LCD_BUS_NAME, LCD_DEVICE_NAME);                             //init  lcd device
    if (EOK != ret) {
        KPrintf("HwLcdInit BoardLcdDevBend error ret %u\n", ret);
        return ERROR;
    }

    Stm32LcdUdelay(5000000);

    HwFsmcInit();

    Stm32LcdUdelay(20000);

    HwLcdInit(&hw_lcd);
#endif

    return ret;
}
