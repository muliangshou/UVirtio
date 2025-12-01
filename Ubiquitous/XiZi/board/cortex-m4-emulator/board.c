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
* @brief support cortex-m4-emulator-board init configure and start-up
* @version 1.0 
* @author fudan
* @date 2021-08-26
*/

/*************************************************
File name: board.c
Description: support cortex-m4-emulator-board init configure and driver/task/... init
Others: 
History: 
1. Date: 2021-04-25
Author: AIIT XUOS Lab
Modification: 
1. support cortex-m4-emulator-board InitBoardHardware
*************************************************/

#include <xizi.h>
#include "hardware_rcc.h"
#include "board.h"
#include "connect_usart.h"
#include "misc.h"
#include <xs_service.h>



static void ClockConfiguration()
{
    int cr,cfgr,pllcfgr;
    int cr1,cfgr1,pllcfgr1;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;
    RCC_HSEConfig(RCC_HSE_ON);
    if (RCC_WaitForHSEStartUp() == SUCCESS)
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div2);
        RCC_PCLK1Config(RCC_HCLK_Div4);
        RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);

        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
    }
    SystemCoreClockUpdate();
}

void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

void  SysTickConfiguration(void)
{
    RCC_ClocksTypeDef  rcc_clocks;
    uint32         cnts;

    RCC_GetClocksFreq(&rcc_clocks);

    cnts = (uint32)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
    cnts = cnts / 8;

    SysTick_Config(cnts);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

void SysTick_Handler(int irqn, void *arg)
{

    TickAndTaskTimesliceUpdate();

}
DECLARE_HW_IRQ(SysTick_IRQn, SysTick_Handler, NONE);


void InitBoardHardware()
{
	int i = 0;
	int ret = 0;
    ClockConfiguration();

    NVIC_Configuration();

    SysTickConfiguration();
#ifdef BSP_USING_UART
	Stm32HwUsartInit();
#endif

    InitBoardMemory((void*)MEMORY_START_ADDRESS, (void*)MEMORY_END_ADDRESS);

#ifdef KERNEL_CONSOLE
    InstallConsole(KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DRV_NAME, KERNEL_CONSOLE_DEVICE_NAME);

    RCC_ClocksTypeDef  rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
    KPrintf("HCLK_Frequency %d, PCLK1_Frequency %d, PCLK2_Frequency %d, SYSCLK_Frequency %d\n", rcc_clocks.HCLK_Frequency, rcc_clocks.PCLK1_Frequency, rcc_clocks.PCLK2_Frequency, rcc_clocks.SYSCLK_Frequency);

    KPrintf("\nconsole init completed.\n");
    KPrintf("board initialization......\n");
#endif
	
}
