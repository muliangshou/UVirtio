/**
* @file arch_interrupt.h
* @brief support rv32m1_vega interrupt
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2022-02-16
*/

#ifndef ARCH_INTERRUPT_H__
#define ARCH_INTERRUPT_H__

#include <RV32M1_ri5cy.h>

#define   ARCH_MAX_IRQ_NUM   NUMBER_OF_INT_VECTORS
#define   ARCH_IRQ_NUM_OFFSET 0

int ArchEnableHwIrq(uint32_t irq_num);
int ArchDisableHwIrq(uint32_t irq_num);

#endif 