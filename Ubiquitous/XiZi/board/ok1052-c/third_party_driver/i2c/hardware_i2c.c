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
* @file hardware_i2c.c
* @brief ok1052-c i2c board relative codes
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-01
*/

#include "fsl_common.h"
#include "fsl_lpi2c.h"

#define I2C_BASE  LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)

#define I2C_CLOCK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))
#define I2C_BAUDRATE 100000U

void I2cHardwareInit(void)
{
    lpi2c_master_config_t masterConfig = {0};

    LPI2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = I2C_BAUDRATE;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(I2C_BASE, &masterConfig, I2C_CLOCK_FREQ);
}

status_t I2cHardwareWrite(LPI2C_Type* base, uint16_t slave_addr, uint32_t subAdd, uint8_t* dataBuff, uint16_t dataLen)
{
    lpi2c_master_transfer_t xfer;
    xfer.slaveAddress = slave_addr;
    xfer.direction = kLPI2C_Write;
    xfer.subaddress = subAdd;
    xfer.subaddressSize = 0x01;
    xfer.data = dataBuff;
    xfer.dataSize = dataLen;
    xfer.flags = kLPI2C_TransferDefaultFlag;
    return LPI2C_MasterTransferBlocking(base, &xfer);
}

status_t I2cHardwareRead(LPI2C_Type* base, uint16_t slave_addr, uint32_t subAdd, uint8_t* dataBuffer, uint16_t dataLen)
{
    lpi2c_master_transfer_t masterXfer = {0};
    masterXfer.slaveAddress = slave_addr;
    masterXfer.direction = kLPI2C_Read;
    masterXfer.subaddress = subAdd;
    masterXfer.subaddressSize = 0x01;
    masterXfer.data = dataBuffer;
    masterXfer.dataSize = dataLen;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;
    return LPI2C_MasterTransferBlocking(base, &masterXfer);
}

