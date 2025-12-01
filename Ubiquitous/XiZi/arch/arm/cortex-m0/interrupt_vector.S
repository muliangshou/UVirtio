//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 10636 of the Stellaris Firmware Development Package.
//
//*****************************************************************************

/**
* @file interrupt_vector.S
* @brief derived from Stellaris Firmware Development Package
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-05-13
*/

/*************************************************
File name: interrupt_vector.S
Description: vector table for a Cortex M3
Others: 
History: 
1. Date: 2021-05-13
Author: AIIT XUOS Lab
Modification: 
1. take startup_gcc.c from revision 10636 of the Stellaris Firmware Development Package for XiZi kernel
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
  .word  _sp
  .word  Reset_Handler
  .word  NMI_Handler                  // NMI_Handler
  .word  HardFaultHandler
  .word  MemFaultHandler              // MemManage_Handler
  .word  BusFault_Handler             // BusFault_Handler
  .word  UsageFault_Handler           // UsageFault_Handler
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry                     // SVC_Handler
  .word  IsrEntry                     // DebugMon_Handler
  .word  IsrEntry
  .word  PendSV_Handler               // PendSV_Handler
  .word  IsrEntry                     // systick 

  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry                     // UART
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
  .word  IsrEntry
