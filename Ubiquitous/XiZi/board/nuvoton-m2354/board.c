/**************************************************************************//**
*
* @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*
* SPDX-License-Identifier: Apache-2.0
*
* Change Logs:
* Date            Author       Notes
* 2020-1-16       Wayne        First version
*
******************************************************************************/

/**
* @file board.c
* @brief define nuvoton-m2354 board init configure and start-up function
* @version 1.1 
* @author AIIT XUOS Lab
* @date 2022-02-24
*/

/*************************************************
File name: board.c
Description: support nuvoton-m2354 board init function
Others: take rt-thread/bsp/nuvoton/libraries/m2354/rtt_port/drv_common.c for references
History: 
1. Date: 2022-02-24
Author: AIIT XUOS Lab
Modification: 
1. support nuvoton-m2354 clock、memory init
2. support nuvoton-m2354 uart driver init
*************************************************/

#include "board.h"
#include <NuMicro.h>
#include <nu_bitutil.h>
#include <nutool_pincfg.h>
#include <nutool_modclkcfg.h>

#include <connect_uart.h>

/* This is the timer interrupt service routine. */
void SysTick_Handler(int irqn, void *arg)
{
    TickAndTaskTimesliceUpdate();
}

/**
 * This function will initial nuvoton-m2354 board.
 */
void InitBoardHardware()
{
    /* Init System/modules clock */
    nutool_modclkcfg_init();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init all pin function setting */
    nutool_pincfg_init();

    /* Configure SysTick */
    SysTick_Config(SystemCoreClock / TICK_PER_SECOND);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Board Memory */
    InitBoardMemory(HEAP_BEGIN, HEAP_END);

#ifdef BSP_USING_UART
    M2354HwUartInit();
#endif

    /* Install Console */
    InstallConsole(KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DRV_NAME, KERNEL_CONSOLE_DEVICE_NAME);

}
