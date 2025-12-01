/****************************************************************************//**
 * @file     startup_M2354.S
 * @version  V1.00
 * @brief    CMSIS Device Startup File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018-2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/**
* @file interrupt_vector.S
* @brief derived from M2354_SERIES_BSP_CMSIS_V3.00.002
* @version 1.1 
* @author AIIT XUOS Lab
* @date 2022-02-23
*/

/*************************************************
File name: interrupt_vector.S
Description: vector table for a M2354KJFAE
Others: 
History: 
1. Date: 2022-02-23
Author: AIIT XUOS Lab
Modification: 
1. take startup_M2354.S for XiUOS
*************************************************/

    .align  2
    .thumb_func
    .global __PC
    .type   __PC, % function

__PC:
    MOV     r0, lr
    BLX     lr
.size   __PC, . - __PC

    .global HardFaultHandler
    .type HardFaultHandler, %function
HardFaultHandler:
    /* get current context */
    MRS     R0, PSP                 /* get fault thread stack pointer */
    PUSH    {LR}
    BL      HwHardFaultException
    POP     {PC}

/*******************************************************************************
*
* The minimal vector table for a Cortex M23. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*******************************************************************************/
.globl InterruptVectors

  .section .isr_vector,"a",%progbits
  .type InterruptVectors, %object
  .size InterruptVectors, . - InterruptVectors

InterruptVectors:
    .long   _sp                   /* Top of Stack */
    .long   Reset_Handler         /* Reset Handler */
    .long   NMI_Handler           /* NMI Handler */
    .long   HardFaultHandler      /* Hard Fault Handler */
    .long   0                     /* Reserved */
    .long   0                     /* Reserved */
    .long   0                     /* Reserved */
    .long   0                     /* Reserved */
    .long   0                     /* Reserved */
    .long   0                     /* Reserved */
    .long   0                     /* Reserved */
    .long   IsrEntry              /* SVCall Handler */
    .long   0                     /* Reserved */
    .long   0                     /* Reserved */
    .long   PendSV_Handler        /* PendSV Handler */
    .long   SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long   IsrEntry /*BOD_IRQHandler         0   */
    .long   IsrEntry /*IRC_IRQHandler         1   */
    .long   IsrEntry /*PWRWU_IRQHandler       2   */
    .long   IsrEntry /*SRAM_IRQHandler        3   */
    .long   IsrEntry /*CLKFAIL_IRQHandler     4   */
    .long   IsrEntry /*Default_Handler        5   */
    .long   IsrEntry /*RTC_IRQHandler         6   */
    .long   IsrEntry /*RTC_TAMPER_IRQHandler  7   */
    .long   IsrEntry /*WDT_IRQHandler         8   */
    .long   IsrEntry /*WWDT_IRQHandler        9   */
    .long   IsrEntry /*EINT0_IRQHandler       10  */
    .long   IsrEntry /*EINT1_IRQHandler       11  */
    .long   IsrEntry /*EINT2_IRQHandler       12  */
    .long   IsrEntry /*EINT3_IRQHandler       13  */
    .long   IsrEntry /*EINT4_IRQHandler       14  */
    .long   IsrEntry /*EINT5_IRQHandler       15  */
    .long   IsrEntry /*GPA_IRQHandler         16  */
    .long   IsrEntry /*GPB_IRQHandler         17  */
    .long   IsrEntry /*GPC_IRQHandler         18  */
    .long   IsrEntry /*GPD_IRQHandler         19  */
    .long   IsrEntry /*GPE_IRQHandler         20  */
    .long   IsrEntry /*GPF_IRQHandler         21  */
    .long   IsrEntry /*QSPI0_IRQHandler       22  */
    .long   IsrEntry /*SPI0_IRQHandler        23  */
    .long   IsrEntry /*BRAKE0_IRQHandler      24  */
    .long   IsrEntry /*EPWM0_P0_IRQHandler    25  */
    .long   IsrEntry /*EPWM0_P1_IRQHandler    26  */
    .long   IsrEntry /*EPWM0_P2_IRQHandler    27  */
    .long   IsrEntry /*BRAKE1_IRQHandler      28  */
    .long   IsrEntry /*EPWM1_P0_IRQHandler    29  */
    .long   IsrEntry /*EPWM1_P1_IRQHandler    30  */
    .long   IsrEntry /*EPWM1_P2_IRQHandler    31  */
    .long   IsrEntry /*TMR0_IRQHandler        32  */
    .long   IsrEntry /*TMR1_IRQHandler        33  */
    .long   IsrEntry /*TMR2_IRQHandler        34  */
    .long   IsrEntry /*TMR3_IRQHandler        35  */
    .long   UART0_IRQHandler /*UART0_IRQHandler       36  */
    .long   IsrEntry /*UART1_IRQHandler       37  */
    .long   IsrEntry /*I2C0_IRQHandler        38  */
    .long   IsrEntry /*I2C1_IRQHandler        39  */
    .long   IsrEntry /*PDMA0_IRQHandler       40  */
    .long   IsrEntry /*DAC_IRQHandler         41  */
    .long   IsrEntry /*EADC0_IRQHandler       42  */
    .long   IsrEntry /*EADC1_IRQHandler       43  */
    .long   IsrEntry /*ACMP01_IRQHandler      44  */
    .long   IsrEntry /*Default_Handler        45  */
    .long   IsrEntry /*EADC2_IRQHandler       46  */
    .long   IsrEntry /*EADC3_IRQHandler       47  */
    .long   IsrEntry /*UART2_IRQHandler       48  */
    .long   IsrEntry /*UART3_IRQHandler       49  */
    .long   IsrEntry /*Default_Handler        50  */
    .long   IsrEntry /*SPI1_IRQHandler        51  */
    .long   IsrEntry /*SPI2_IRQHandler        52  */
    .long   IsrEntry /*USBD_IRQHandler        53  */
    .long   IsrEntry /*USBH_IRQHandler        54  */
    .long   IsrEntry /*USBOTG_IRQHandler      55  */
    .long   IsrEntry /*CAN0_IRQHandler        56  */
    .long   IsrEntry /*Default_Handler        57  */
    .long   IsrEntry /*SC0_IRQHandler         58  */
    .long   IsrEntry /*SC1_IRQHandler         59  */
    .long   IsrEntry /*SC2_IRQHandler         60  */
    .long   IsrEntry /*Default_Handler        61  */
    .long   IsrEntry /*SPI3_IRQHandler        62  */
    .long   IsrEntry /*Default_Handler        63  */
    .long   IsrEntry /*SDH0_IRQHandler        64  */
    .long   IsrEntry /*Default_Handler        65  */
    .long   IsrEntry /*Default_Handler        66  */
    .long   IsrEntry /*Default_Handler        67  */
    .long   IsrEntry /*I2S0_IRQHandler        68  */
    .long   IsrEntry /*Default_Handler        69  */
    .long   IsrEntry /*OPA0_IRQHandler        70  */
    .long   IsrEntry /*CRPT_IRQHandler        71  */
    .long   IsrEntry /*GPG_IRQHandler         72  */
    .long   IsrEntry /*EINT6_IRQHandler       73  */
    .long   IsrEntry /*UART4_IRQHandler       74  */
    .long   IsrEntry /*UART5_IRQHandler       75  */
    .long   IsrEntry /*USCI0_IRQHandler       76  */
    .long   IsrEntry /*USCI1_IRQHandler       77  */
    .long   IsrEntry /*BPWM0_IRQHandler       78  */
    .long   IsrEntry /*BPWM1_IRQHandler       79  */
    .long   IsrEntry /*Default_Handler        80  */
    .long   IsrEntry /*Default_Handler        81  */
    .long   IsrEntry /*I2C2_IRQHandler        82  */
    .long   IsrEntry /*Default_Handler        83  */
    .long   IsrEntry /*QEI0_IRQHandler        84  */
    .long   IsrEntry /*QEI1_IRQHandler        85  */
    .long   IsrEntry /*ECAP0_IRQHandler       86  */
    .long   IsrEntry /*ECAP1_IRQHandler       87  */
    .long   IsrEntry /*GPH_IRQHandler         88  */
    .long   IsrEntry /*EINT7_IRQHandler       89  */
    .long   IsrEntry /*Default_Handler        90  */
    .long   IsrEntry /*Default_Handler        91  */
    .long   IsrEntry /*Default_Handler        92  */
    .long   IsrEntry /*Default_Handler        93  */
    .long   IsrEntry /*Default_Handler        94  */
    .long   IsrEntry /*Default_Handler        95  */
    .long   IsrEntry /*Default_Handler        96  */
    .long   IsrEntry /*Default_Handler        97  */
    .long   IsrEntry /*PDMA1_IRQHandler       98  */
    .long   IsrEntry /*SCU_IRQHandler         99  */
    .long   IsrEntry /*LCD_IRQHandler         100 */
    .long   IsrEntry /*TRNG_IRQHandler        101 */
    .long   IsrEntry /*Default_Handler        102 */
    .long   IsrEntry /*Default_Handler        103 */
    .long   IsrEntry /*Default_Handler        104 */
    .long   IsrEntry /*Default_Handler        105 */
    .long   IsrEntry /*Default_Handler        106 */
    .long   IsrEntry /*Default_Handler        107 */
    .long   IsrEntry /*Default_Handler        108 */
    .long   IsrEntry /*KS_IRQHandler          109 */ 
    .long   IsrEntry /*TAMPER_IRQHandler      110 */ 
    .long   IsrEntry /*EWDT_IRQHandler        111 */ 
    .long   IsrEntry /*EWWDT_IRQHandler       112 */ 
    .long   IsrEntry /*NS_ISP_IRQHandler      113 */ 
    .long   IsrEntry /*TMR4_IRQHandler        114 */ 
    .long   IsrEntry /*TMR5_IRQHandler        115 */

    .end

