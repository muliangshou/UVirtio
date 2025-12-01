/*
 * Copyright (c) 2022 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     SummerGift   add spi flash port file
 */

/**
* @file connect_flash_spi.c
* @brief support ok1052-c board spi flash function and register to bus framework
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-01
*/

/*************************************************
File name: connect_flash_spi.c
Description: support ok1052-c board spi flash bus register function
Others: take RT-Thread v4.0.2/bsp/stm32/stm32f407-atk-explorer/board/ports/spi-flash-init.c
                https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History:
1. Date: 2022-03-01
Author: AIIT XUOS Lab
Modification:
1. for ok1052-c board compilation
*************************************************/

#include "flash_spi.h"

int FlashW25qxxSpiDeviceInit(void)
{
#ifdef BSP_USING_SPI1

#endif
    return EOK;
}