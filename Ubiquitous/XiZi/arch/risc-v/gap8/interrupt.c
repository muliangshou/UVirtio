/****************************************************************************
 * arch/risc-v/src/gap8/gap8_interrupt.c
 * GAP8 event system
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 *  GAP8 features a FC controller and a 8-core cluster. IRQ from peripherals
 *  have unique ID, which are dispatched to the FC or cluster by the SOC
 *  event unit, and then by the FC event unit or cluster event unit, and
 *  finally to FC or cluster. Peripherals share the same IRQ entry.
 ****************************************************************************/

/**
* @file interrupt.c
* @brief support gap8 interrupt enable and disable
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-09-02
*/

/*************************************************
File name: interrupt.c
Description: support gap8 interrupt enable and disable
Others: take nuttx/arch/risc-v/gap8/gap8_interrupt.c  for references
                https://github.com/apache/incubator-nuttx.git
History: 
1. Date: 2021-09-02
Author: AIIT XUOS Lab
Modification: modify interrupt enable/disable function and add interrupt process function
*************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch_interrupt.h>
#include <xs_base.h>
#include <xs_isr.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Function exported to the NuttX kernel */

void up_mdelay(unsigned int time)
{
  while (time--)
    {
      volatile int dummy = 200000;
      while (dummy--)
        {
        }
    }
}

/****************************************************************************
 * Name: ArchDisableHwIrq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'. Mind the Machine privilege.
 *
 ****************************************************************************/

int ArchDisableHwIrq(uint32_t irq_num)
{
  FCEU->MASK_IRQ_AND = (1UL << irq_num);
}
/****************************************************************************
 * Name: ArchEnableHwIrq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'. Mind the Machine privilege.
 *
 ****************************************************************************/

int ArchEnableHwIrq(uint32_t irq_num)
{
  FCEU->MASK_IRQ_OR = (1 << irq_num);
}


x_base DisableLocalInterrupt(void)
{
    x_base level;
    asm volatile("nop");
    asm volatile ("csrrci %0, mstatus, 8" : "=r"(level));

    return level;
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
  x_base newstat;

  asm volatile ("csrw mstatus, %0" :: "r"(oldstat));
}

/****************************************************************************
 * Name: gap8_sleep_wait_sw_evnt
 *
 * Description:
 *   Sleep on specific event.
 *
 ****************************************************************************/

void gap8_sleep_wait_sw_evnt(uint32_t event_mask)
{
  FCEU->MASK_OR = event_mask;
  // __builtin_pulp_event_unit_read((void *)&FCEU->EVENT_WAIT_CLEAR, 0);
  FCEU->MASK_AND = event_mask;
}

/****************************************************************************
 * Name: irqinitialize
 *
 * Description:
 *   Initialize the IRQ on FC.
 *
 ****************************************************************************/
extern void gap8_udma_doirq(int irq, void *arg);
void irqinitialize(void)
{
  /* Deactivate all the soc events */

  SOC_EU->FC_MASK_MSB = 0xffffffff;
  SOC_EU->FC_MASK_LSB = 0xffffffff;

  /* enable soc peripheral interrupt */

  isrManager.done->registerIrq(GAP8_IRQ_FC_UDMA, gap8_udma_doirq, NONE);
  isrManager.done->enableIrq(GAP8_IRQ_FC_UDMA);

}

/****************************************************************************
 * Name: gap8_dispatch_irq
 *
 * Description:
 *   Called from IRQ vectors. Input vector id. Return SP pointer, modified
 *   or not.
 *
 ****************************************************************************/
void *gap8_dispatch_irq(uint32_t vector, void *current_regs)
{
  /* Clear pending bit and trigger a software event.
   * GAP8 would sleep on sw event 3 on up_idle().
   */

  FCEU->BUFFER_CLEAR = (1 << vector);
  EU_SW_EVNT_TRIG->TRIGGER_SET[3] = 0;

  g_current_regs = current_regs;


  isrManager.done->incCounter();
  isrManager.done->handleIrq(vector);

  isrManager.done->decCounter();
}


