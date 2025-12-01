/****************************************************************************//**
 * @file     startup_M2354.S
 * @version  V1.00
 * @brief    CMSIS Device Startup File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018-2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/**
* @file boot.S
* @brief derived from M2354_SERIES_BSP_CMSIS_V3.00.002
* @version 1.1 
* @author AIIT XUOS Lab
* @date 2022-02-23
*/

/*************************************************
File name: boot.S
Description: Reset and init function
Others: 
History: 
1. Date: 2022-02-23
Author: AIIT XUOS Lab
Modification: 
1. take startup_M2354.S for XiUOS
*************************************************/


    .syntax unified
    .arch   armv8 - m.base
    .fpu softvfp
    .thumb

/* start address for the initialization values of the .data section.
defined in linker script */
.word   _sidata
/* start address for the .data section. defined in linker script */
.word   _sdata
/* end address for the .data section. defined in linker script */
.word   _edata
/* start address for the .bss section. defined in linker script */
.word   _sbss
/* end address for the .bss section. defined in linker script */
.word   _ebss

    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function

Reset_Handler:

    /* Check SecureWorld */
    MOV     R0, R15
    LSLS    R0, R0, #3
    BMI.N   GotoSystemInit

    /* Unlock Register */
    LDR     R0, =0x40000100
    LDR     R1, =0x59
    STR     R1, [R0]
    LDR     R1, =0x16
    STR     R1, [R0]
    LDR     R1, =0x88
    STR     R1, [R0]

    /* power gating */
    /* M32(0x400001f4) = 0xfffffffful; */
    LDR     R0, =0x400001f4
    LDR     R1, =0xffffffff
    STR     R1, [R0]

    /* M32(0x400000dC) = 0ul; */
    LDR     R0, =0x400000dC
    LDR     R1, =0x0
    STR     R1, [R0]

    /* Enable GPIO clks, SRAM clks, Trace clk */
    /* CLK->AHBCLK |= (0xffful << 20) | (1ul << 14); */

    LDR     R0, =0x40000200
    LDR     R1, [R0,#0x4]

    LDR     R2, =0xfff02000

    ORRS    R1, R1, R2
    STR     R1, [R0,#0x4]

GotoSystemInit:

    /* Lock register */
    LDR     R0, =0x40000100
    MOVS    R1, #0
    STR     R1, [R0]

/* Copy the data segment initializers from flash to SRAM */  
    movs r1, #0
    b LoopCopyDataInit

CopyDataInit:
    ldr r3, =_sidata
    ldr r3, [r3, r1]
    str r3, [r0, r1]
    adds r1, r1, #4

LoopCopyDataInit:
    ldr r0, =_sdata
    ldr r3, =_edata
    adds r2, r0, r1
    cmp r2, r3
    bcc CopyDataInit
    ldr r2, =_sbss
    b LoopFillZerobss

/* Zero fill the bss segment. */  
FillZerobss:
    movs r3, #0
    str r3, [r2, #4]
    adds r2, r2, #4

LoopFillZerobss:
    ldr r3, = _ebss
    cmp r2, r3
    bcc FillZerobss
/* Call the clock system intitialization function.*/
    bl  SystemInit

/* Call the application's entry point.*/
    bl  entry
    bx  lr
.size Reset_Handler, .-Reset_Handler
