/**
* @file interrupt.c
* @brief support gap8 interrupt enable and disable
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-09-02
*/


#include <arch_interrupt.h>
#include <xs_base.h>
#include <xs_isr.h>
#include <core_feature_base.h>

int ArchDisableHwIrq(uint32_t irq_num)
{
    ECLIC_DisableIRQ(irq_num);
    return 0;
}

int ArchEnableHwIrq(uint32_t irq_num)
{
    ECLIC_EnableIRQ(irq_num);
    return 0;
}


x_base DisableLocalInterrupt(void)
{
   return __RV_CSR_READ_CLEAR(CSR_MSTATUS, MSTATUS_MIE);
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
  __RV_CSR_WRITE(CSR_MSTATUS, oldstat);
}


// extern  void KTaskOsAssignAfterIrq(void *context);

// void IsrEntry()
// {
//     uint32 ipsr;

//     isrManager.done->incCounter();
//     isrManager.done->handleIrq(ipsr);
//     KTaskOsAssignAfterIrq(NONE);
//     isrManager.done->decCounter();
    
// }



