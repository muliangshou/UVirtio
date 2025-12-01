/*
* Copyright (c) 2020 AIIT XUOS Lab
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
 * @file hs300x.h
 * @brief hs300x sensor driver
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.03.17
 */

#ifndef __INCLUDE_NUTTX_SENSORS_HS300X_H
#define __INCLUDE_NUTTX_SENSORS_HS300X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_HS300X_I2C)

/* HS300X Registers frequency and addresses*/

#ifndef CONFIG_HS300X_I2C_FREQUENCY
#  define CONFIG_HS300X_I2C_FREQUENCY 400000
#endif

#ifndef CONFIG_SENSOR_DEVICE_HS300X_I2C_ADDR
#  define CONFIG_SENSOR_DEVICE_HS300X_I2C_ADDR 0x44
#endif


/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: hs300x_register
 *
 * Description:
 *   Register the HS300x character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/i2c1"
 *   i2c - An instance of the I2C interface to use to communicate with HS300X
 *   addr - The I2C address of the HS300x.  The base I2C address of the HS300X
 *   is 0x44. 
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hs300x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                  uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_HS300X_I2C */
#endif /* __INCLUDE_NUTTX_SENSORS_HS300X_H */
