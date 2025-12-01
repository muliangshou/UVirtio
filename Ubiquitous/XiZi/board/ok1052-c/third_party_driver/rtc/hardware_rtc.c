/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file hardware_rtc.c
 * @brief I2C RTC drivers
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.1.18
 */

/*************************************************
File name: hardware_rtc.c
Description: support ok1052-c board rtc hardware driver I2C function

History:
1. Date: 2022-01-18
Author: AIIT XUOS Lab
Modification:
1. support ok1052-c board rtc
*************************************************/

#include "connect_rtc.h"
#include "connect_i2c.h"

void RtcI2cInit()
{
    I2cHardwareInit();
}

status_t RtcI2cWrite(LPI2C_Type *base, uint32_t sub_addr, uint8_t *buf, uint16_t size)
{
    return I2cHardwareWrite(base, I2C_RTC_ADDR, sub_addr, buf, size);
}

uint32_t RtcI2cRead(LPI2C_Type *base,uint32_t sub_addr,uint8_t* buf, uint16_t size)
{
    return I2cHardwareRead(base, I2C_RTC_ADDR, sub_addr, buf, size);
}

