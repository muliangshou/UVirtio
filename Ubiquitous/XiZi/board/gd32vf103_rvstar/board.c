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
* @brief support rvstar init configure and start-up
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-09-02
*/

#include <board.h>
#include <xizi.h>
#include <device.h>
#include <nuclei_sdk_soc.h>
// #include <gd32vf103.h>
#include <core_feature_timer.h>

extern void entry(void);
extern int InitHwUart();

#ifndef configKERNEL_INTERRUPT_PRIORITY
#define configKERNEL_INTERRUPT_PRIORITY         0
#endif

void xPortTaskSwitch(void)
{
    /* Clear Software IRQ, A MUST */
    SysTimer_ClearSWIRQ();
    isrManager.done->setSwitchTrigerFlag();
}

void vPortSetupTimerInterrupt(void)
{
    uint64_t ticks = SYSTICK_TICK_CONST;

    /* Make SWI and SysTick the lowest priority interrupts. */
    /* Stop and clear the SysTimer. SysTimer as Non-Vector Interrupt */
    SysTick_Config(ticks);
    ECLIC_DisableIRQ(SysTimer_IRQn);
    ECLIC_SetLevelIRQ(SysTimer_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    ECLIC_SetShvIRQ(SysTimer_IRQn, ECLIC_NON_VECTOR_INTERRUPT);
    ECLIC_EnableIRQ(SysTimer_IRQn);

    /* Set SWI interrupt level to lowest level/priority, SysTimerSW as Vector Interrupt */
    ECLIC_SetShvIRQ(SysTimerSW_IRQn, ECLIC_VECTOR_INTERRUPT);
    ECLIC_SetLevelIRQ(SysTimerSW_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    ECLIC_EnableIRQ(SysTimerSW_IRQn);
}

void Gd32vf103Start(void)
{
    entry();
}
void InitBoardHardware(void)
{

    vPortSetupTimerInterrupt();


    InitBoardMemory(MEMORY_START_ADDRESS, MEMORY_END_ADDRESS);

    InitHwUart();
    InstallConsole("uart4", "uart4_drv", "uart4_dev4");

	KPrintf("console init completed.\n");
    KPrintf("board initialization......\n");
    
    // InitHwTick();
    KPrintf("memory address range: [0x%08x - 0x%08x], size: %d\n", (x_ubase) MEMORY_START_ADDRESS, (x_ubase) MEMORY_END_ADDRESS, gd32vf103_SRAM_SIZE);
    /* initialize memory system */
	
    KPrintf("board init done.\n");
	KPrintf("start kernel...\n");
    return;
}

