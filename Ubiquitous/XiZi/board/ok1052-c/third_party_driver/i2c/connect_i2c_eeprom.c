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
* @file connect_i2c_eeprom.h
* @brief ok1052-c board eeprom relative codes
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-01
*/

#include "board.h"
#include "connect_i2c.h"
#include "fsl_lpi2c.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define I2C_EEPROM_BASE  LPI2C1
#define I2C_EEPROM_ADDR (0xA0 >> 1)

/*******************************************************************************
 * Code
 ******************************************************************************/

void I2cEEpromTestWrite(void)
{
    uint8_t dat[8] = {0};

    if(I2cHardwareRead(I2C_EEPROM_BASE, I2C_EEPROM_ADDR, 0, dat, 8) == kStatus_Success)
    {
        i2c_print("Read from EEPROM %d %d %d %d %d %d %d %d\r\n",
            dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7]);
    }

    for(uint8_t i = 0; i < 8; i++)
    {
        dat[i] ++;
    }

    if(I2cHardwareWrite(I2C_EEPROM_BASE, I2C_EEPROM_ADDR, 0, dat, 8) == kStatus_Success)
    {
        i2c_print("Write  to EEPROM %d %d %d %d %d %d %d %d\r\n",
            dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7]);
    }

    memset(dat, 0, 8);
    if(I2cHardwareRead(I2C_EEPROM_BASE, I2C_EEPROM_ADDR, 0, dat, 8) == kStatus_Success)
    {
        i2c_print("Read from EEPROM %d %d %d %d %d %d %d %d\r\n",
            dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7]);
    }
}

int I2cEEpromTest(void)
{
    BOARD_InitI2C1Pins();
    I2cHardwareInit();
    I2cEEpromTestWrite();
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)| SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)| SHELL_CMD_PARAM_NUM(0),
                   eeprom, I2cEEpromTest, test i2c eeprom);

