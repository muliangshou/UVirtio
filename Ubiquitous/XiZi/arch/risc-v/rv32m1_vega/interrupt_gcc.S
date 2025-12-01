/**
* @file interrupt_gcc.S
* @brief support vega interrupt
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2022-02-16
*/

#include "boot.h"

  .section      .text.entry
  .align 2
  .global IRQ_Handler
IRQ_Handler:

    SAVE_X_REGISTERS

    mv  fp, sp

    /* switch to interrupt stack */
    # la    sp, __stack

    /* interrupt handle */

    csrr  a0, mcause
    csrr  a1, mepc
    mv    a2, sp
    call  SystemIrqHandler

    /* switch to from thread stack */

    mv  sp, fp
    mv  a0, fp
    call KTaskOsAssignAfterIrq
    j SwitchKTaskContextExit

