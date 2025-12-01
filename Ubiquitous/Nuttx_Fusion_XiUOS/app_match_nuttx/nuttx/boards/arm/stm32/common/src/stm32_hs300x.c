/****************************************************************************
 * boards/arm/stm32/common/src/stm32_hs300x.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/hs300x.h>

#include "stm32.h"
#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_hs300x_initialize
 *
 * Description:
 *   Initialize and register the hs300x Temperature Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number
 *   busno - The I2C bus number
 *   used to build the device path as /dev/i2c1_dev0
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_hs300x_initialize(int devno, int busno)
{
  FAR struct i2c_master_s *i2c;
  char devpath[20];
  int ret;

  sninfo("Initializing HS300x!\n");

  /* Get an instance of the I2C interface */
  i2c =  stm32_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the temperature sensor,such as /dev/i2c1_dev0. */
  snprintf(devpath, 20, "/dev/i2c%d_dev%d", busno, devno);
  ret = hs300x_register(devpath, i2c, CONFIG_SENSOR_DEVICE_HS300X_I2C_ADDR);
  if (ret < 0)
    {
      stm32_i2cbus_uninitialize(i2c);
    }

  return ret;
}
