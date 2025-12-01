/**
* @file interrupt.c
* @brief support interrupt
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-09-02
*/


#include <arch_interrupt.h>
#include <xs_base.h>
#include <xs_isr.h>
#include "fsl_common.h"
#include <RV32M1_ri5cy.h>

int ArchDisableHwIrq(uint32_t irq_num)
{
    DisableIRQ(irq_num);
    return 0;
}

int ArchEnableHwIrq(uint32_t irq_num)
{
    EnableIRQ(irq_num);
    return 0;
}


x_base DisableLocalInterrupt(void)
{
  __disable_irq();
  return 0;
}

/****************************************************************************
 * Name: EnableLocalInterrupt
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

void EnableLocalInterrupt(x_base oldstat)
{
  __enable_irq();
}

typedef void (*irq_handler_t)(void);
extern const irq_handler_t isrTable[];

void SystemIrqHandler(uint32_t mcause)
{
    uint32_t intNum;

    if (mcause & 0x80000000) /* For external interrupt. */
    {
        intNum = mcause & 0x1FUL;

        /* Clear pending flag in EVENT unit .*/
        EVENT_UNIT->INTPTPENDCLEAR = (1U << intNum);

        /* Now call the real irq handler for intNum */
        isrManager.done->incCounter();
        isrTable[intNum]();
        isrManager.done->decCounter();
    }
}


