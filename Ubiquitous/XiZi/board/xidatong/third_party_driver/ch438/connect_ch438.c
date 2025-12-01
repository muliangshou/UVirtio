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
* @file connect_ch438.c
* @brief support to register ch438 pointer and function
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-03-17
*/

#include <connect_ch438.h>

static const uint8 offset_addr[] = {0x00,0x10,0x20,0x30,0x08,0x18,0x28,0x38,};		/* uart offset address*/
static const uint8 interrupt_num[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,};	/* SSR register data*/
static BusType ch438_pin;
static int ch438_sem = NONE;

static void Ch438Irq(void *parameter)
{
	KSemaphoreAbandon(ch438_sem);
}

/**
 * The time delay function.
 *
 * @param microseconds.
 */
static void ImxrtUdelay(uint32 us)
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
                break;
            }
        }
    }
}

void CH438SetOutput(void)
{
	struct PinParam pin_cfg;	
	int ret = 0;

	struct BusConfigureInfo configure_info;
	configure_info.configure_cmd = OPE_CFG;
	configure_info.private_data = (void *)&pin_cfg;

    pin_cfg.cmd = GPIO_CONFIG_MODE;
    pin_cfg.pin = CH438_D0_PIN;
    pin_cfg.mode = GPIO_CFG_OUTPUT;

	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D0_PIN pin %d failed!\n", CH438_D0_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D1_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D1_PIN pin %d failed!\n", CH438_D1_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D2_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D2_PIN pin %d failed!\n", CH438_D2_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D3_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D3_PIN pin %d failed!\n", CH438_D3_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D4_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D4_PIN pin %d failed!\n", CH438_D4_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D5_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D5_PIN pin %d failed!\n", CH438_D5_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D6_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D6_PIN pin %d failed!\n", CH438_D6_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D7_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D7_PIN pin %d failed!\n", CH438_D7_PIN);
        return ;
    }
}

void CH438SetInput(void)
{
	struct PinParam pin_cfg;	
	int ret = 0;

	struct BusConfigureInfo configure_info;
	configure_info.configure_cmd = OPE_CFG;
	configure_info.private_data = (void *)&pin_cfg;

    pin_cfg.cmd = GPIO_CONFIG_MODE;
    pin_cfg.pin = CH438_D0_PIN;
    pin_cfg.mode = GPIO_CFG_INPUT_PULLUP;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D0_PIN pin %d INPUT_PULLUP failed!\n", CH438_D0_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D1_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D1_PIN pin %d INPUT_PULLUP failed!\n", CH438_D1_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D2_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D2_PIN pin %d INPUT_PULLUP failed!\n", CH438_D2_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D3_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D3_PIN pin %d INPUT_PULLUP failed!\n", CH438_D3_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D4_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D4_PIN pin %d INPUT_PULLUP failed!\n", CH438_D4_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D5_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D5_PIN pin %d INPUT_PULLUP failed!\n", CH438_D5_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D6_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D6_PIN pin %d INPUT_PULLUP failed!\n", CH438_D6_PIN);
        return ;
    }

	pin_cfg.pin = CH438_D7_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config CH438_D7_PIN pin %d INPUT_PULLUP failed!\n", CH438_D7_PIN);
        return ;
    }
}

/*********************************************************************************************************
** Function name: ReadCH438Data
** Function: read data from CH438
** input: address
**
** output: data
**         
** date: 2011.8.26
**-------------------------------------------------------------------------------------------------------
** modify:
** date:
**-------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint8 ReadCH438Data( uint8 addr )
{
//read data api

//ALE WR RD is high level when not busy, when reading data, put address on the serial port and put ALE low level for a few clock

//switch input pin, put RD low level, delay a few clock, read the data from the serial port

//put RD and ALE high level
	uint8 dat;
	struct PinStat pin_stat;
	struct BusBlockWriteParam write_param;
	struct BusBlockReadParam read_param;
	write_param.buffer = (void *)&pin_stat;
	read_param.buffer = (void *)&pin_stat;
	
	pin_stat.val = GPIO_HIGH;

    pin_stat.pin = CH438_NWR_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	pin_stat.pin = CH438_NRD_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	pin_stat.pin = CH438_ALE_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	CH438SetOutput();
	ImxrtUdelay(1);
	
	if (addr &0x80) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D7_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D7_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x40) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D6_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D6_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x20) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D5_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D5_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x10) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D4_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D4_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x08) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D3_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D3_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x04) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D2_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D2_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x02) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D1_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D1_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x01) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D0_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D0_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	ImxrtUdelay(1);

	pin_stat.val = GPIO_LOW;
    pin_stat.pin = CH438_ALE_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	ImxrtUdelay(1);		

	CH438SetInput();
	ImxrtUdelay(1);

	pin_stat.val = GPIO_LOW;
    pin_stat.pin = CH438_NRD_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	
	ImxrtUdelay(1);	
	
	dat = 0;

    pin_stat.pin = CH438_D7_PIN;
    if(BusDevReadData(ch438_pin->owner_haldev, &read_param)) 
		dat |= 0x80;

	pin_stat.pin = CH438_D6_PIN;
	if (BusDevReadData(ch438_pin->owner_haldev, &read_param))	
		dat |= 0x40;

	pin_stat.pin = CH438_D5_PIN;
	if (BusDevReadData(ch438_pin->owner_haldev, &read_param))	
		dat |= 0x20;

	pin_stat.pin = CH438_D4_PIN;
	if (BusDevReadData(ch438_pin->owner_haldev, &read_param))	
		dat |= 0x10;

	pin_stat.pin = CH438_D3_PIN;
	if (BusDevReadData(ch438_pin->owner_haldev, &read_param))	
		dat |= 0x08;

	pin_stat.pin = CH438_D2_PIN;
	if (BusDevReadData(ch438_pin->owner_haldev, &read_param))	
		dat |= 0x04;

	pin_stat.pin = CH438_D1_PIN;
	if (BusDevReadData(ch438_pin->owner_haldev, &read_param))	
		dat |= 0x02;

	pin_stat.pin = CH438_D0_PIN;
	if (BusDevReadData(ch438_pin->owner_haldev, &read_param))	
		dat |= 0x01;
	
	pin_stat.val = GPIO_HIGH;
    pin_stat.pin = CH438_NRD_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	pin_stat.val = GPIO_HIGH;
    pin_stat.pin = CH438_ALE_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	ImxrtUdelay(1);

	return dat;
}
	
void WriteCH438Data(uint8 addr, uint8 dat)
{
	struct PinStat pin_stat;
	struct BusBlockWriteParam write_param;
	write_param.buffer = (void *)&pin_stat;

	pin_stat.val = GPIO_HIGH;
    pin_stat.pin = CH438_ALE_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	pin_stat.val = GPIO_HIGH;
    pin_stat.pin = CH438_NRD_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	pin_stat.val = GPIO_HIGH;
    pin_stat.pin = CH438_NWR_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);	

	CH438SetOutput();
	ImxrtUdelay(1);	
	
	if (addr &0x80) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D7_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D7_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x40) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D6_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D6_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x20) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D5_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D5_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x10) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D4_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D4_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x08) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D3_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D3_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x04) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D2_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D2_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x02) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D1_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D1_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (addr &0x01) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D0_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D0_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	ImxrtUdelay(1);

	pin_stat.val = GPIO_LOW;
    pin_stat.pin = CH438_ALE_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	ImxrtUdelay(1);

	if (dat &0x80) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D7_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D7_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (dat &0x40) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D6_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D6_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (dat &0x20) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D5_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D5_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (dat &0x10) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D4_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D4_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (dat &0x08) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D3_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D3_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (dat &0x04) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D2_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D2_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (dat &0x02) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D1_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D1_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	if (dat &0x01) {
		pin_stat.val = GPIO_HIGH;
    	pin_stat.pin = CH438_D0_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	} else {
		pin_stat.val = GPIO_LOW;
    	pin_stat.pin = CH438_D0_PIN;
    	BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	}

	ImxrtUdelay(1);

	pin_stat.val = GPIO_LOW;
    pin_stat.pin = CH438_NWR_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	ImxrtUdelay(1);

	pin_stat.val = GPIO_HIGH;
    pin_stat.pin = CH438_NWR_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	pin_stat.val = GPIO_HIGH;
    pin_stat.pin = CH438_ALE_PIN;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);
	
	ImxrtUdelay(1);	

	CH438SetInput();

	return;
}

/*********************************************************************************************************
** Function name: WriteCH438Block
** Function: write data to CH438
** input: CH438 register address, data block length, data cache address
**
** output: 
**         
** date: 2011.8.26
**-------------------------------------------------------------------------------------------------------
** modify:
** date:
**-------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void WriteCH438Block(uint8 maddr, uint8 mlen, uint8 *mbuf)   
{
	while (mlen--) {
		WriteCH438Data(maddr, *mbuf++);
	}
}

/*********************************************************************************************************
** Function name: Ch438UartSend
** Function: active FIFO mode, CH438 send multibyte data by uart 0, max length is 128 bytes a single time
** input: send data cache address, send data length
**
** output: 
**
** date: 2011.8.26.
**-------------------------------------------------------------------------------------------------------
** modify:
** date:
**-------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Ch438UartSend(uint8 ext_uart_no, uint8 *data, uint16 Num)
{
	uint8 REG_LSR_ADDR,REG_THR_ADDR;
	
	REG_LSR_ADDR = offset_addr[ext_uart_no] | REG_LSR0_ADDR;
	REG_THR_ADDR = offset_addr[ext_uart_no] | REG_THR0_ADDR;
			
    while (1) {
        while((ReadCH438Data(REG_LSR_ADDR) & BIT_LSR_TEMT) == 0);    /* wait for sending data done, THR and TSR is NULL */

        if (Num <= 128) {
            WriteCH438Block(REG_THR_ADDR, Num, data);
            break;
        } else {
            WriteCH438Block(REG_THR_ADDR, 128, data);
            Num -= 128;
            data += 128;
        }
    }
}

/*********************************************************************************************************
** Function name: Ch438UartRcv
** Function: forbidden FIFO mode, CH438 receive multibyte data from uart 0
** input: recv data address
**
** output: recv data length
**
** date: 2011.8.26
**-------------------------------------------------------------------------------------------------------
** modify:
** date:
**-------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint8 Ch438UartRcv(uint8 ext_uart_no, uint8 *buf, x_size_t size)
{
    uint8 rcv_num = 0;
	uint8 dat = 0;
	uint8 REG_LSR_ADDR,REG_RBR_ADDR;
	uint8 *read_buffer;
	x_size_t buffer_index = 0;
	
	read_buffer = buf;
	
	REG_LSR_ADDR = offset_addr[ext_uart_no] | REG_LSR0_ADDR;
	REG_RBR_ADDR = offset_addr[ext_uart_no] | REG_RBR0_ADDR;
			
	while ((ReadCH438Data(REG_LSR_ADDR) & BIT_LSR_DATARDY) == 0);    /* wait for data is ready */

    while (((ReadCH438Data(REG_LSR_ADDR) & BIT_LSR_DATARDY) == 0x01) && (size != 0)) {
		dat = ReadCH438Data(REG_RBR_ADDR);

		*read_buffer = dat;
		read_buffer++;
		buffer_index++;

		if (CH438_BUFFSIZE == buffer_index) {
			buffer_index = 0;
			read_buffer = buf;
		}
			
        rcv_num = rcv_num + 1;
		--size;
    }

    return rcv_num;
}

static void Timeout438Proc(void *parameter) 
{
	uint8_t	rbr,lsr;
	
	while( ( ReadCH438Data( REG_LSR0_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR0_ADDR );
		KPrintf("0.RBR=%02x\r\n",rbr);
	}

	while( ( ReadCH438Data( REG_LSR1_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR1_ADDR );
		KPrintf("1.RBR=%02x\r\n",rbr);
	}

	while( ( ReadCH438Data( REG_LSR2_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR2_ADDR );
		KPrintf("2.RBR=%02x\r\n",rbr);
	}

	while( ( ReadCH438Data( REG_LSR3_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR3_ADDR );
		KPrintf("3.RBR=%02x\r\n",rbr);
	}

	while( ( ReadCH438Data( REG_LSR4_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR4_ADDR );
		KPrintf("4.RBR=%02x\r\n",rbr);
	}

	while( ( ReadCH438Data( REG_LSR5_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR5_ADDR );
		KPrintf("5.RBR=%02x\r\n",rbr);
	}

	while( ( ReadCH438Data( REG_LSR6_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR6_ADDR );
		KPrintf("6.RBR=%02x\r\n",rbr);
	}

	while( ( ReadCH438Data( REG_LSR7_ADDR ) & BIT_LSR_DATARDY ) == 0x01 ) {
		rbr =  ReadCH438Data( REG_RBR7_ADDR );
		KPrintf("7.RBR=%02x\r\n",rbr);
	}
}

void Ch438PortInit(uint8 ext_uart_no, uint32 BaudRate )
{
	uint32 div;
	uint8 DLL,DLM,dlab;
	uint8 REG_LCR_ADDR;
	uint8 REG_DLL_ADDR;
	uint8 REG_DLM_ADDR;
	uint8 REG_IER_ADDR;
	uint8 REG_MCR_ADDR;
	uint8 REG_FCR_ADDR;
	uint8 REG_RBR_ADDR;
	uint8 REG_THR_ADDR;
	uint8 REG_IIR_ADDR;
	
	REG_LCR_ADDR = offset_addr[ext_uart_no] | REG_LCR0_ADDR;
	REG_DLL_ADDR = offset_addr[ext_uart_no] | REG_DLL0_ADDR;
	REG_DLM_ADDR = offset_addr[ext_uart_no] | REG_DLM0_ADDR;
	REG_IER_ADDR = offset_addr[ext_uart_no] | REG_IER0_ADDR;
	REG_MCR_ADDR = offset_addr[ext_uart_no] | REG_MCR0_ADDR;
	REG_FCR_ADDR = offset_addr[ext_uart_no] | REG_FCR0_ADDR;
	REG_RBR_ADDR = offset_addr[ext_uart_no] | REG_RBR0_ADDR;
	REG_THR_ADDR = offset_addr[ext_uart_no] | REG_THR0_ADDR;
	REG_IIR_ADDR = offset_addr[ext_uart_no] | REG_IIR0_ADDR;
			
    WriteCH438Data(REG_IER_ADDR, BIT_IER_RESET);             /* reset the uart */
	//MdelayKTask(50);
	ImxrtUdelay(50000);
	
	dlab = ReadCH438Data(REG_IER_ADDR);
	dlab &= 0xDF;
	WriteCH438Data(REG_IER_ADDR, dlab);
	
	dlab = ReadCH438Data(REG_LCR_ADDR);
	dlab |= 0x80;		//set LCR register DLAB bit 1
	WriteCH438Data(REG_LCR_ADDR, dlab);

    div = (Fpclk >> 4) / BaudRate;
    DLM = div >> 8;
    DLL = div & 0xff;
	WriteCH438Data(REG_DLL_ADDR, DLL);             /* set bps */
    WriteCH438Data(REG_DLM_ADDR, DLM);
	WriteCH438Data(REG_FCR_ADDR, BIT_FCR_RECVTG1 | BIT_FCR_RECVTG0 | BIT_FCR_FIFOEN);    /* set FIFO mode, 112 bytes */

    WriteCH438Data(REG_LCR_ADDR, BIT_LCR_WORDSZ1 | BIT_LCR_WORDSZ0);                     /* 8 bit word size, 1 bit stop bit, no crc */

    WriteCH438Data(REG_IER_ADDR, BIT_IER_IERECV);    /* enable interrupt */

    WriteCH438Data(REG_MCR_ADDR, BIT_MCR_OUT2);// | BIT_MCR_RTS     | BIT_MCR_DTR 	);              /* allow interrupt output, DTR and RTS is 1 */

	WriteCH438Data(REG_FCR_ADDR, ReadCH438Data(REG_FCR_ADDR)| BIT_FCR_TFIFORST);  /* release the data in FIFO */
}

void CH438PortInitParityCheck(uint8 ext_uart_no, uint32 BaudRate)
{
	uint32 div;
	uint8 DLL,DLM,dlab;
	uint8 REG_LCR_ADDR;
	uint8 REG_DLL_ADDR;
	uint8 REG_DLM_ADDR;
	uint8 REG_IER_ADDR;
	uint8 REG_MCR_ADDR;
	uint8 REG_FCR_ADDR;
	uint8 REG_RBR_ADDR;
	uint8 REG_THR_ADDR;
	uint8 REG_IIR_ADDR;
	
	REG_LCR_ADDR = offset_addr[ext_uart_no] | REG_LCR0_ADDR;
	REG_DLL_ADDR = offset_addr[ext_uart_no] | REG_DLL0_ADDR;
	REG_DLM_ADDR = offset_addr[ext_uart_no] | REG_DLM0_ADDR;
	REG_IER_ADDR = offset_addr[ext_uart_no] | REG_IER0_ADDR;
	REG_MCR_ADDR = offset_addr[ext_uart_no] | REG_MCR0_ADDR;
	REG_FCR_ADDR = offset_addr[ext_uart_no] | REG_FCR0_ADDR;
	REG_RBR_ADDR = offset_addr[ext_uart_no] | REG_RBR0_ADDR;
	REG_THR_ADDR = offset_addr[ext_uart_no] | REG_THR0_ADDR;
	REG_IIR_ADDR = offset_addr[ext_uart_no] | REG_IIR0_ADDR;
			
    WriteCH438Data(REG_IER_ADDR, BIT_IER_RESET);             /* reset the uart */
	//MdelayKTask(50);
	ImxrtUdelay(50000);	
	
	dlab = ReadCH438Data(REG_IER_ADDR);
	dlab &= 0xDF;
	WriteCH438Data(REG_IER_ADDR, dlab);
	
	dlab = ReadCH438Data(REG_LCR_ADDR);
	dlab |= 0x80;		//set LCR register DLAB bit 1
	WriteCH438Data(REG_LCR_ADDR, dlab);

    div = (Fpclk >> 4) / BaudRate;
    DLM = div >> 8;
    DLL = div & 0xff;
	WriteCH438Data(REG_DLL_ADDR, DLL);             /* set bps */
    WriteCH438Data(REG_DLM_ADDR, DLM);
	WriteCH438Data(REG_FCR_ADDR, BIT_FCR_RECVTG1 | BIT_FCR_FIFOEN);    /* set FIFO mode, 64 bytes */

	WriteCH438Data(REG_LCR_ADDR, BIT_LCR_WORDSZ1 | BIT_LCR_WORDSZ0 | BIT_LCR_PAREN | BIT_LCR_PARMODE0);		/* 8 bit work size, 1 bit stop bit, even parity check */

    WriteCH438Data(REG_IER_ADDR, BIT_IER_IERECV);    /* enable interrupt */

    WriteCH438Data(REG_MCR_ADDR, BIT_MCR_OUT2);// | BIT_MCR_RTS     | BIT_MCR_DTR 	);              /* allow interrupt output, DTR and RTS is 1 */

	WriteCH438Data(REG_FCR_ADDR, ReadCH438Data(REG_FCR_ADDR)| BIT_FCR_TFIFORST);  /* release the data in FIFO */
}

static uint32 ImxrtCh438Configure(struct SerialCfgParam *ext_serial_cfg)
{
	NULL_PARAM_CHECK(ext_serial_cfg);

	switch (ext_serial_cfg->data_cfg.port_configure)
	{
		case PORT_CFG_INIT:
			Ch438PortInit(ext_serial_cfg->data_cfg.ext_uart_no, ext_serial_cfg->data_cfg.serial_baud_rate);
			break;
		case PORT_CFG_PARITY_CHECK:
			CH438PortInitParityCheck(ext_serial_cfg->data_cfg.ext_uart_no, ext_serial_cfg->data_cfg.serial_baud_rate);
			break;
		default:
			KPrintf("ImxrtCh438Configure do not support configure %d\n", ext_serial_cfg->data_cfg.port_configure);
			return ERROR;
			break;
	}

	return EOK;
}

static uint32 ImxrtCh438Init(struct SerialDriver *serial_drv,  struct SerialCfgParam *ext_serial_cfg)
{
	NULL_PARAM_CHECK(serial_drv);
	
	struct PinParam pin_cfg;
	struct PinStat pin_stat;
	int ret = 0;

	struct BusConfigureInfo configure_info;
	configure_info.configure_cmd = OPE_CFG;
	configure_info.private_data = (void *)&pin_cfg;

	struct BusBlockWriteParam write_param;
	write_param.buffer = (void *)&pin_stat;

	ch438_pin = PinBusInitGet();

	CH438SetOutput();

	/* config NWR pin as output*/
    pin_cfg.cmd = GPIO_CONFIG_MODE;
    pin_cfg.pin = CH438_NWR_PIN;
    pin_cfg.mode = GPIO_CFG_OUTPUT;

	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config NWR pin %d failed!\n", CH438_NWR_PIN);
        return ERROR;
    }

	/* config NRD pin as output*/
	pin_cfg.pin = CH438_NRD_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config NRD pin %d failed!\n", CH438_NRD_PIN);
        return ERROR;
    }

	/* config ALE pin as output*/
	pin_cfg.pin = CH438_ALE_PIN;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config ALE pin %d failed!\n", CH438_ALE_PIN);
        return ERROR;
    }

	/* set NWR pin as high*/
    pin_stat.pin = CH438_NWR_PIN;
    pin_stat.val = GPIO_HIGH;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	/* set NRD pin as high*/
    pin_stat.pin = CH438_NRD_PIN;
    pin_stat.val = GPIO_HIGH;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	/* set ALE pin as high*/
    pin_stat.pin = CH438_ALE_PIN;
    pin_stat.val = GPIO_HIGH;
    BusDevWriteData(ch438_pin->owner_haldev, &write_param);

	return ImxrtCh438Configure(ext_serial_cfg);
}

static uint32 Ch438DrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;

    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;
	struct SerialCfgParam *ext_serial_cfg = (struct SerialCfgParam *)configure_info->private_data;

    switch (configure_info->configure_cmd)
    {
		case OPE_INT:
			ret = ImxrtCh438Init(serial_drv, ext_serial_cfg);
			break;
		default:
			break;
    }

    return ret;
}

static uint32 ImxrtCh438WriteData(void *dev, struct BusBlockWriteParam *write_param)
{
	NULL_PARAM_CHECK(dev);
	NULL_PARAM_CHECK(write_param);

	int write_len, write_len_continue;
	int i, write_index;
	uint8 *write_buffer;

	struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)dev;
	struct SerialDevParam *dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

	write_len = write_param->size;
	write_len_continue = write_param->size;
	write_buffer = (uint8 *)write_param->buffer;

	if (write_len > 256) {
		if (0 == write_len % 256) {
			write_index = write_len / 256;
			for (i = 0; i < write_index; i ++) {
				Ch438UartSend(dev_param->ext_uart_no, write_buffer + i * 256, 256);
			}
		} else {
			write_index = 0;
			while (write_len_continue > 256) {
				Ch438UartSend(dev_param->ext_uart_no, write_buffer + write_index * 256, 256);
				write_index++;
				write_len_continue = write_len - write_index * 256;
			}
			Ch438UartSend(dev_param->ext_uart_no, write_buffer + write_index * 256, write_len_continue);
		}
	} else {
		Ch438UartSend(dev_param->ext_uart_no, write_buffer, write_len);
	}

	return EOK;
}

static uint32 ImxrtCh438ReadData(void *dev, struct BusBlockReadParam *read_param)
{
	NULL_PARAM_CHECK(dev);
	NULL_PARAM_CHECK(read_param);

	x_err_t result;
	uint8 rcv_num = 0;
	uint8 gInterruptStatus;
	uint8 interrupt_done = 0;
	uint8 InterruptStatus;
	static uint8 dat;
	uint8 REG_LCR_ADDR;
	uint8 REG_DLL_ADDR;
	uint8 REG_DLM_ADDR;
	uint8 REG_IER_ADDR;
	uint8 REG_MCR_ADDR;
	uint8 REG_FCR_ADDR;
	uint8 REG_RBR_ADDR;
	uint8 REG_THR_ADDR;
	uint8 REG_IIR_ADDR;
	uint8 REG_LSR_ADDR;
	uint8 REG_MSR_ADDR;

	struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)dev;
	struct SerialDevParam *dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

	while (!interrupt_done) {
		result = KSemaphoreObtain(ch438_sem, WAITING_FOREVER);
		if (EOK == result) {
			gInterruptStatus = ReadCH438Data(REG_SSR_ADDR);
			if (!gInterruptStatus) { 
				// dat = ReadCH438Data(REG_LCR0_ADDR);	
				// dat = ReadCH438Data(REG_IER0_ADDR);	
				// dat = ReadCH438Data(REG_MCR0_ADDR);	
				// dat = ReadCH438Data(REG_LSR0_ADDR);	
				// dat = ReadCH438Data(REG_MSR0_ADDR);	
				// dat = ReadCH438Data(REG_RBR0_ADDR);	
				// dat = ReadCH438Data(REG_THR0_ADDR);	
				// dat = ReadCH438Data(REG_IIR0_ADDR);	
				// dat = dat;	
				interrupt_done = 0;
			} else {
				if (gInterruptStatus & interrupt_num[dev_param->ext_uart_no]) {   /* check which uart port triggers interrupt*/
					REG_LCR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_LCR0_ADDR;
					REG_DLL_ADDR = offset_addr[dev_param->ext_uart_no] | REG_DLL0_ADDR;
					REG_DLM_ADDR = offset_addr[dev_param->ext_uart_no] | REG_DLM0_ADDR;
					REG_IER_ADDR = offset_addr[dev_param->ext_uart_no] | REG_IER0_ADDR;
					REG_MCR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_MCR0_ADDR;
					REG_FCR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_FCR0_ADDR;
					REG_RBR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_RBR0_ADDR;
					REG_THR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_THR0_ADDR;
					REG_IIR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_IIR0_ADDR;
					REG_LSR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_LSR0_ADDR;
					REG_MSR_ADDR = offset_addr[dev_param->ext_uart_no] | REG_MSR0_ADDR;				
								
					InterruptStatus = ReadCH438Data( REG_IIR_ADDR ) & 0x0f;    /* read the status of the uart port*/

					if ((INT_RCV_OVERTIME == InterruptStatus) || (INT_RCV_SUCCESS == InterruptStatus)) {
						rcv_num = Ch438UartRcv(dev_param->ext_uart_no, (uint8 *)read_param->buffer, read_param->size);
						read_param->read_length = rcv_num;

						interrupt_done = 1;

						// int i;
						// uint8 *buffer = (uint8 *)read_param->buffer;
						// for (i = 0; i < rcv_num; i ++) {
						// 	KPrintf("Ch438UartRcv i %u data 0x%x\n", i, buffer[i]);
						// }
					}
				}
			}
		}
	}
	return rcv_num;
}

static const struct SerialDevDone dev_done = 
{
    NONE,
    NONE,
    ImxrtCh438WriteData,
    ImxrtCh438ReadData,
};

static void Ch438InitDefault(struct SerialDriver *serial_drv)
{
	struct PinParam pin_cfg;
	BusType ch438_pin;

    struct BusConfigureInfo configure_info;

    int ret = 0;
	configure_info.configure_cmd = OPE_CFG;
	configure_info.private_data = (void *)&pin_cfg;

	ch438_sem = KSemaphoreCreate(0);
	if (ch438_sem < 0) {
		KPrintf("Ch438InitDefault create sem failed .\n");
		return;
	}

	ch438_pin = PinBusInitGet();

	pin_cfg.cmd = GPIO_CONFIG_MODE;
    pin_cfg.pin = CH438_INT_PIN;
    pin_cfg.mode = GPIO_CFG_INPUT_PULLUP;
    ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config BSP_CH438_INT_PIN pin %d failed!\n", CH438_INT_PIN);
        return;
    }

	pin_cfg.cmd = GPIO_IRQ_REGISTER;
    pin_cfg.pin = CH438_INT_PIN;
    pin_cfg.irq_set.irq_mode = GPIO_IRQ_EDGE_FALLING;
    pin_cfg.irq_set.hdr = Ch438Irq;
    pin_cfg.irq_set.args = NONE;
	ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config BSP_CH438_INT_PIN GPIO_IRQ_REGISTER %d failed!\n", CH438_INT_PIN);
        return;
    }

	//disable ch438 int gpio irq
	pin_cfg.cmd = GPIO_IRQ_DISABLE;
    pin_cfg.pin = CH438_INT_PIN;
    ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config BSP_CH438_INT_PIN GPIO_IRQ_DISABLE %d failed!\n", CH438_INT_PIN);
        return;
    }

	//enable ch438 int gpio irq
	pin_cfg.cmd = GPIO_IRQ_ENABLE;
    pin_cfg.pin = CH438_INT_PIN;
    ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config BSP_CH438_INT_PIN GPIO_IRQ_ENABLE %d failed!\n", CH438_INT_PIN);
        return;
    }
}

static uint32 ImxrtCh438DevRegister(struct SerialHardwareDevice *serial_dev, char *dev_name)
{
	x_err_t ret = EOK;
	serial_dev->dev_done = &dev_done;
	serial_dev->ext_serial_mode = RET_TRUE;
    ret = SerialDeviceRegister(serial_dev, NONE, dev_name);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init Serial device %s register error %d\n", dev_name, ret);
        return ERROR;
    }

    ret = SerialDeviceAttachToBus(dev_name, CH438_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init Serial device %s register error %d\n", dev_name, ret);
        return ERROR;
    }

	return ret;
}

int Imxrt1052HwCh438Init(void)
{
    static struct SerialBus serial_bus;
    static struct SerialDriver serial_drv;

    x_err_t ret = EOK;
    
    ret = SerialBusInit(&serial_bus, CH438_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init Serial bus init error %d\n", ret);
        return ERROR;
    }

    serial_drv.configure = Ch438DrvConfigure;
    ret = SerialDriverInit(&serial_drv, CH438_DRIVER_NAME);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init Serial driver init error %d\n", ret);
        return ERROR;
    }

    ret = SerialDriverAttachToBus(CH438_DRIVER_NAME, CH438_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init Serial driver attach error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_0;
	static struct SerialDevParam dev_param_0;
	dev_param_0.ext_uart_no = 0;
	serial_dev_0.haldev.private_data = (void *)&dev_param_0;
	ret = ImxrtCh438DevRegister(&serial_dev_0, CH438_DEVICE_NAME_0);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_1;
	static struct SerialDevParam dev_param_1;
	dev_param_1.ext_uart_no = 1;
	serial_dev_1.haldev.private_data = (void *)&dev_param_1;
	ret = ImxrtCh438DevRegister(&serial_dev_1, CH438_DEVICE_NAME_1);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_2;
	static struct SerialDevParam dev_param_2;
	dev_param_2.ext_uart_no = 2;
	serial_dev_2.haldev.private_data = (void *)&dev_param_2;
	ret = ImxrtCh438DevRegister(&serial_dev_2, CH438_DEVICE_NAME_2);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_3;
	static struct SerialDevParam dev_param_3;
	dev_param_3.ext_uart_no = 3;
	serial_dev_3.haldev.private_data = (void *)&dev_param_3;
	ret = ImxrtCh438DevRegister(&serial_dev_3, CH438_DEVICE_NAME_3);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_4;
	static struct SerialDevParam dev_param_4;
	dev_param_4.ext_uart_no = 4;
	serial_dev_4.haldev.private_data = (void *)&dev_param_4;
	ret = ImxrtCh438DevRegister(&serial_dev_4, CH438_DEVICE_NAME_4);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_5;
	static struct SerialDevParam dev_param_5;
	dev_param_5.ext_uart_no = 5;
	serial_dev_5.haldev.private_data = (void *)&dev_param_5;
	ret = ImxrtCh438DevRegister(&serial_dev_5, CH438_DEVICE_NAME_5);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_6;
	static struct SerialDevParam dev_param_6;
	dev_param_6.ext_uart_no = 6;
	serial_dev_6.haldev.private_data = (void *)&dev_param_6;
	ret = ImxrtCh438DevRegister(&serial_dev_6, CH438_DEVICE_NAME_6);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	static struct SerialHardwareDevice serial_dev_7;
	static struct SerialDevParam dev_param_7;
	dev_param_7.ext_uart_no = 7;
	serial_dev_7.haldev.private_data = (void *)&dev_param_7;
	ret = ImxrtCh438DevRegister(&serial_dev_7, CH438_DEVICE_NAME_7);
    if (ret != EOK) {
        KPrintf("ImxrtHwCh438Init ImxrtCh438DevRegister error %d\n", ret);
        return ERROR;
    }

	Ch438InitDefault(&serial_drv);

    return ret;
}

#ifdef CH438_EXTUART_TEST
static void CH438RegTest(unsigned char num)//for test
{
	uint8 dat;
	
	KPrintf("current test serial num:  %02x \r\n",offset_addr[num]);
	KPrintf("IER: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_IER0_ADDR));//IER
	KPrintf("IIR: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_IIR0_ADDR));//IIR
	KPrintf("LCR: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_LCR0_ADDR));//LCR
	KPrintf("MCR: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_MCR0_ADDR));//MCR
	KPrintf("LSR: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_LSR0_ADDR));//LSR
	KPrintf("MSR: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_MSR0_ADDR));//MSR
	KPrintf("FCR: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_FCR0_ADDR));//FCR
	KPrintf("SSR: 0x%02x\r\n",ReadCH438Data(offset_addr[num] | REG_SSR_ADDR ));//SSR
	
	KPrintf("SCR0: 0x%02x\r\n",(unsigned short)ReadCH438Data(offset_addr[num] | REG_SCR0_ADDR));//SCR
	dat = 0x55;
	WriteCH438Data(offset_addr[num] | REG_SCR0_ADDR, dat);
	KPrintf("SCR55: 0x%02x\r\n",(unsigned short)ReadCH438Data(offset_addr[num] | REG_SCR0_ADDR));//SCR
	dat = 0xAA;
	WriteCH438Data(offset_addr[num] | REG_SCR0_ADDR, dat);
	KPrintf("SCRAA: 0x%02x\r\n",(unsigned short)ReadCH438Data(offset_addr[num] | REG_SCR0_ADDR));//SCR
}

static struct Bus *bus;
static struct HardwareDev *dev;
static struct Driver *drv;

static void Ch438Read(void *parameter)
{
	uint8 RevLen;
    uint8 ext_uart_no = 0;
	uint8 i, cnt = 0;

    struct BusBlockReadParam read_param;
    static uint8 Ch438Buff[8][256];

    struct BusBlockWriteParam write_param;
	
    while (1)
    {
        KPrintf("ready to read test_ch438 data\n");

        read_param.buffer = Ch438Buff[ext_uart_no];
        RevLen = BusDevReadData(dev, &read_param);

		KPrintf("test_ch438 get data %u\n", RevLen);
							
		if (RevLen) {
			for(i = 0 ; i < RevLen; i ++) {
				KPrintf("i %u data 0x%x\n", i, Ch438Buff[ext_uart_no][i]);
				Ch438Buff[ext_uart_no][i] = 0;
			}

			cnt ++;
			write_param.buffer = &cnt;
			write_param.size = 1;
			BusDevWriteData(dev, &write_param);
		}

	}
}

static void TestCh438Init(void)
{ 
    x_err_t flag;

    struct BusConfigureInfo configure_info;

    bus = BusFind(CH438_BUS_NAME);
    drv = BusFindDriver(bus, CH438_DRIVER_NAME);

    dev = BusFindDevice(bus, CH438_DEVICE_NAME_0);

    struct SerialCfgParam serial_cfg;
    memset(&serial_cfg, 0, sizeof(struct SerialCfgParam));
    configure_info.configure_cmd = OPE_INT;
    configure_info.private_data = (void *)&serial_cfg;

    serial_cfg.data_cfg.port_configure = PORT_CFG_INIT;

    serial_cfg.data_cfg.ext_uart_no = 0;
    serial_cfg.data_cfg.serial_baud_rate = 115200;
    BusDrvConfigure(drv, &configure_info);

	KPrintf("ready to create test_ch438 task\n");

	int32 task_CH438_read = KTaskCreate("task_CH438_read", Ch438Read, NONE, 2048, 10); 
	flag = StartupKTask(task_CH438_read);
    if (flag != EOK) {
		KPrintf("StartupKTask task_CH438_read failed .\n");
		return;
	} 
}

void TestCh438(void)
{
	TestCh438Init();

	CH438RegTest(0);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                                                TestCh438, TestCh438, TestCh438 );

#endif												
