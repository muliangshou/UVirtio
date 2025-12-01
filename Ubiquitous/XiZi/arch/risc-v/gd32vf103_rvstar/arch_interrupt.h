#ifndef ARCH_INTERRUPT_H__
#define ARCH_INTERRUPT_H__

#include <nuclei_sdk_soc.h>

#define   ARCH_MAX_IRQ_NUM   128
#define ARCH_IRQ_NUM_OFFSET 0

int ArchEnableHwIrq(uint32_t irq_num);
int ArchDisableHwIrq(uint32_t irq_num);

#endif 