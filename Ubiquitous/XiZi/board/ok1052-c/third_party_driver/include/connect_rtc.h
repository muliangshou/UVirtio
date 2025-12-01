/*
* Copyright (c) 2022 AIIT XUOS Lab
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
* @file connect_rtc.h
* @brief define ok1052-c board rtc function and struct
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-01
*/

#ifndef __CONNECT_RTC_H_
#define __CONNECT_RTC_H_

#include "fsl_common.h"
#include "fsl_lpi2c.h"

#define I2C_RTC_BASE LPI2C1
#define I2C_RTC_ADDR 0x32

void RtcI2cInit(void);
status_t RtcI2cWrite(LPI2C_Type *base, uint32_t sub_addr, uint8_t *buf, uint16_t size);
uint32_t RtcI2cRead(LPI2C_Type *base, uint32_t sub_addr, uint8_t *buf, uint16_t size);

int Imxrt1052HwRtcInit(void);

#endif

