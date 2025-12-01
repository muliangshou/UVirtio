/****************************************************************************
 * arch/risc-v/src/gap8/gap8_tim.c
 * GAP8 basic timer
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

/**
* @file: hardware_hwtimer.c
* @brief: support gap8 tick interrupt
* @version: 1.0
* @author:  AIIT XUOS Lab
* @date:    2021-09-02
*/

/*************************************************
File name: hardware_hwtimer.c
Description: support gap8 tick interrupt
Others: 
History: 
1. Date: 2021-09-02
Author: AIIT XUOS Lab
Modification: 
    add tick interrupt isr
*************************************************/

/****************************************************************************
 *  FC core has a 64-bit basic timer, able to split into 2 32-bit timers,
 *  with identicle memory map and 2 IRQ channels, for both FC core and
 *  cluster. We would use it as system timer.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <xs_isr.h>
#include <gap8_tim.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gap8_tim_instance
{
  basic_tim_reg_t *reg;
  uint32_t core_clock;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gap8_tim_instance fc_basic_timer =
{
  .reg = BASIC_TIM,
  .core_clock = 200000000,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gap8_timisr
 *
 * Description:
 *   Timer ISR to perform RR context switch
 *
 ****************************************************************************/
extern int TickIsr(void);
void gap8_timisr(int irq, void *arg)
{
  TickIsr();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_initialize
 *
 * Description:
 *   Initialize the timer based on the frequency of source clock and ticks
 *   per second.
 *
 ****************************************************************************/

void timer_initialize(void)
{
  /* Set input clock to 1MHz. FC won't exceed 250MHz */

  uint32_t prescaler = (fc_basic_timer.core_clock / 1000000) & 0xff;
  uint32_t cmpval = TICK_PER_SECOND;

  /* Initialize timer registers */

  fc_basic_timer.reg->CMP_LO = cmpval;
  fc_basic_timer.reg->CFG_REG_LO = (prescaler << 8) |
    BASIC_TIM_CLKSRC_FLL | BASIC_TIM_PRESC_ENABLE | BASIC_TIM_MODE_CYCL |
    BASIC_TIM_IRQ_ENABLE | BASIC_TIM_RESET | BASIC_TIM_ENABLE;
  fc_basic_timer.reg->VALUE_LO = 0;

  isrManager.done->registerIrq(GAP8_IRQ_FC_TIMER_LO, gap8_timisr, NONE);
  isrManager.done->enableIrq(GAP8_IRQ_FC_TIMER_LO);
}
