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
* @brief support gap8 init configure and start-up
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-09-02
*/

#include <board.h>
#include <arch_interrupt.h>
#include <xizi.h>
#include <device.h>

extern void entry(void);
extern int InitHwUart(void);
extern void irqinitialize(void);
extern void timer_initialize(void);
extern void gapuino_sysinit(void);

extern unsigned int __bss_end__;
/* Return core id. */
uint32_t _procid( void )
{
    uint32_t ulProcid = 0;
    __asm__ volatile( "csrr %0, mhartid" : "=r"( ulProcid ) );
    ulProcid  = ulProcid & 0xf;
    return ( ulProcid );
}

/* Return cluster id. */
uint32_t _clusterid( void )
{
    uint32_t ulCluster = 0;
    __asm__ volatile( "csrr %0, mhartid" : "=r"( ulCluster ) );
    ulCluster  = ulCluster >> 5;
    return ( ulCluster );
}

void GapuinoStart(uint32_t mhartid)
{
    gapuino_sysinit();
    entry();
}
void InitBoardHardware(void)
{
    irqinitialize();
    
    InitBoardMemory(MEMORY_START_ADDRESS, MEMORY_END_ADDRESS);

    InitHwUart();
    InstallConsole("uart0", "uart0_drv", "uart0_dev0");

	KPrintf("console init completed.\n");
    KPrintf("board initialization......\n");
    
    timer_initialize();
    KPrintf("memory address range: [0x%08x - 0x%08x], size: %d\n", (x_ubase) MEMORY_START_ADDRESS, (x_ubase) MEMORY_END_ADDRESS, GAP8_SRAM_SIZE);
    /* initialize memory system */
    KPrintf("gap8 cluster id: [0x%x], _procid: [0x%x]\n", _clusterid(),_procid());
	
    KPrintf("board init done.\n");
	KPrintf("start kernel...\n");
    return;
}

