/**
  *************** (C) COPYRIGHT 2017 STMicroelectronics ************************
  * @file      startup_stm32f103xb.s
  * @author    MCD Application Team
  * @brief     STM32F103xB Devices vector table for Atollic toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Configure the clock system   
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M3 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/**
* @file interrupt_vector_nano.S
* @brief derived from Stellaris Firmware Development Package
* @version 1.1 
* @author AIIT XUOS Lab
* @date 2021-12-02
*/

/*************************************************
File name: interrupt_vector_nano.S
Description: vector table for a stm32f103xb
Others: 
History: 
1. Date: 2021-12-02
Author: AIIT XUOS Lab
Modification: 
1. take startup_stm32f103xb.s for XiZi kernel reference
*************************************************/


//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
.globl  InterruptVectors

/******************************************************************************
*******************************************************************************/
  .section  .isr_vector,"a",%progbits
  .type  InterruptVectors, %object
  .size  InterruptVectors, .-InterruptVectors

InterruptVectors:
  .word _sp
  .word Reset_Handler
  .word NMI_Handler
  .word HardFaultHandler
  .word MemFaultHandler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word IsrEntry
  .word IsrEntry
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word UartIsr1
  .word IsrEntry              //UartIsr2
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word IsrEntry
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
