/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author       Notes
 * 2010-01-25   Bernard      first version
 * 2012-06-01   aozima       set pendsv priority to 0xFF.
 * 2012-08-17   aozima       fixed bug: store r8 - r11.
 * 2013-02-20   aozima       port to gcc.
 * 2013-06-18   aozima       add restore MSP feature.
 * 2013-11-04   bright       fixed hardfault bug for gcc.
 * 2019-03-31   xuzhuoyi     port to Cortex-M23.
 */

/*************************************************
File name: pendsv.S
Description: PendSV interrupt handler
Others: take RT-Thread v4.0.2/libcpu/arm/cortex-m23/context_gcc.S for references
        https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History: 
1. Date: 2022-02-28
Author: AIIT XUOS Lab
*************************************************/

#include <xsconfig.h>

.cpu cortex-m23
.syntax unified
.thumb
.text

.equ    SCB_VTOR,           0xE000ED08
.equ    NVIC_INT_CTRL,      0xE000ED04
.equ    NVIC_SYSPRI2,       0xE000ED20
.equ    NVIC_PENDSV_PRI,    0xFFFF0000
.equ    NVIC_PENDSVSET,     0x10000000

/* R0 --> switch from thread stack
 * R1 --> switch to thread stack
 * psr, pc, LR, R12, R3, R2, R1, R0 are pushed into [from] stack
 */
    .global PendSV_Handler
    .type PendSV_Handler, %function
PendSV_Handler:
    /* disable interrupt to protect context switch */
    MRS     R2, PRIMASK
    CPSID   I

    LDR     R0, =KtaskSwitchInterruptFlag
    LDR     R1, [R0]
    CMP     R1, #0x00
    BEQ     pendsv_exit     /* pendsv aLReady handled */

    /* clear KtaskSwitchInterruptFlag to 0 */
    MOVS    R1, #0
    STR     R1, [R0]

    LDR     R0, =InterruptFromKtask
    LDR     R1, [R0]
    CMP     R1, #0x00
    BEQ     switch_to_task    /* skip register save at the first time */

    MRS     R1, PSP                 /* get from thread stack pointer */

    SUBS    R1, R1, #0x20           /* space for {R4 - R7} and {R8 - R11} */
    LDR     R0, [R0]
    STR     R1, [R0]                /* update from thread stack pointer */

    STMIA   R1!, {R4 - R7}          /* push thread {R4 - R7} register to thread stack */

    MOV     R4, R8                  /* mov thread {R8 - R11} to {R4 - R7} */
    MOV     R5, R9
    MOV     R6, R10
    MOV     R7, R11
    STMIA   R1!, {R4 - R7}          /* push thread {R8 - R11} high register to thread stack */
switch_to_task:
    BL       UpdateRunningTask

    LDR     R1, =InterruptToKtask
    LDR     R1, [R1]
    LDR     R1, [R1]                /* load thread stack pointer */

    LDMIA   R1!, {R4 - R7}          /* pop thread {R4 - R7} register from thread stack */
    PUSH    {R4 - R7}               /* push {R4 - R7} to MSP for copy {R8 - R11} */

    LDMIA   R1!, {R4 - R7}          /* pop thread {R8 - R11} high register from thread stack to {R4 - R7} */
    MOV     R8,  R4                 /* mov {R4 - R7} to {R8 - R11} */
    MOV     R9,  R5
    MOV     R10, R6
    MOV     R11, R7

    POP     {R4 - R7}               /* pop {R4 - R7} from MSP */

    MSR     PSP, R1                 /* update stack pointer */

pendsv_exit:
    /* restore interrupt */
    MSR     PRIMASK, R2

    MOVS    R0, #0x03
    RSBS    R0, R0, #0x00
    BX      R0
    