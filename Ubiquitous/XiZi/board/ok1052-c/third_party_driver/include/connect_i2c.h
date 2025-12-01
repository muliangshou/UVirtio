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
* @file connect_i2c.h
* @brief define ok1052-c board i2c function and struct
* @version 1.0
* @author AIIT XUOS Lab
* @date 2021-04-25
*/

#ifndef CONNECT_I2C_H
#define CONNECT_I2C_H

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct Stm32I2c
{
    LPI2C_Type* base;
    uint16_t slave_addr;
    uint32_t sub_addr;
}Stm32I2cType;

#define i2c_print KPrintf

int Imxrt1052HwI2cInit(void);

#ifdef __cplusplus
}
#endif

#endif
