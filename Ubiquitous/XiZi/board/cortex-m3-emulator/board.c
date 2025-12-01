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
* @file board.c
* @brief support cortex-m3-emulator init configure and start-up
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-05-13
*/


#include <board.h>
#include <xizi.h>
#include <device.h>
#include <arch_interrupt.h>

void SysTick_Handler(int irqn, void *arg)
{
    TickAndTaskTimesliceUpdate();
}
DECLARE_HW_IRQ(SYSTICK_IRQN, SysTick_Handler, NONE);

void InitBoardHardware()
{
	InitBoardMemory((void*)LM3S_SRAM_START, (void*)LM3S_SRAM_END);

	extern int InitHwUart(void);
	InitHwUart();
	
	InstallConsole(SERIAL_BUS_NAME_1, SERIAL_DRV_NAME_1, SERIAL_DEVICE_NAME_1);
#ifdef DEBUG_VIRTIO_SERIAL
    InstallVirtioSerial(VIRTIO_BUS_NAME, VIRTIO_DRV_NAME, VIRTIO_DEVICE_NAME);
#endif
}
