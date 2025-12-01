/*
* Copyright (c) 2021 AIIT XUOS Lab
* XiUOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
* @file connect_spi.h
* @brief define ok1052-c board spi function and struct
* @version 1.0
* @author AIIT XUOS Lab
* @date 2021-04-25
*/

#ifndef CONNECT_SPI_H
#define CONNECT_SPI_H

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_USING_RX_DMA_FLAG   (1<<0)
#define SPI_USING_TX_DMA_FLAG   (1<<1)

struct Stm32HwSpi
{
    LPSPI_Type *base;
    uint8_t irq;
    uint8_t mode;
    void *priv_data;
};

struct Stm32Spi
{
    LPSPI_Type *instance;
    char *bus_name;
    struct SpiBus spi_bus;
};

int Imxrt1052HwSpiInit(void);
x_err_t HwSpiDeviceAttach(const char *bus_name, const char *device_name);

#ifdef __cplusplus
}
#endif

#endif
