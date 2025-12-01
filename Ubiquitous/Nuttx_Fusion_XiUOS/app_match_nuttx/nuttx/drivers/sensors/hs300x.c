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
 * @file hs300x.c
 * @brief hs300x sensor driver
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.03.17
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/hs300x.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_HS300X_I2C)

/****************************************************************************
 * Private
 ****************************************************************************/
struct hs300x_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */
static int     hs300x_i2c_write(FAR struct hs300x_dev_s *priv, FAR char *buffer, int buflen);
static int     hs300x_i2c_read(FAR struct hs300x_dev_s *priv, FAR char *buffer, int buflen);

/* Character driver methods */
static int     hs300x_open(FAR struct file *filep);
static int     hs300x_close(FAR struct file *filep);
static ssize_t hs300x_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t hs300x_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hs300xfops =
{
  hs300x_open,
  hs300x_close,
  hs300x_read,
  hs300x_write,
  NULL,
  NULL,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hs300x_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int hs300x_i2c_write(FAR struct hs300x_dev_s *priv,
                          FAR char *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */
  msg.frequency = CONFIG_HS300X_I2C_FREQUENCY,
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */
  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: hs300x_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int hs300x_i2c_read(FAR struct hs300x_dev_s *priv,
                         FAR char *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_HS300X_I2C_FREQUENCY,
  msg.addr      = priv->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = (FAR uint8_t *)buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */
  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}


/****************************************************************************
 * Name: hs300x_open
 ****************************************************************************/
static int hs300x_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: hs300x_close
 ****************************************************************************/

static int hs300x_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: hs300x_read
 ****************************************************************************/

static ssize_t hs300x_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct hs300x_dev_s *priv   = inode->i_private;
  int                    ret;

  if (buflen != 4)
    {
      snerr("ERROR: You can't read something other than 32 bits (4 bytes)\n");
      return -1;
    }
  /* Restart and read 32-bits from the register */
  ret = hs300x_i2c_read(priv, buffer, 4);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return  (ssize_t)ret;;
    }

  return buflen;
}

/****************************************************************************
 * Name: hs300x_write
 ****************************************************************************/

static ssize_t hs300x_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct hs300x_dev_s *priv   = inode->i_private;
  int                    ret;
    
  ret = hs300x_i2c_write(priv, buffer, buflen);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return  (ssize_t)ret;;
    }

  return buflen;
}


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
                  uint8_t addr)
{
  FAR struct hs300x_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_SENSOR_DEVICE_HS300X_I2C_ADDR);

  /* Initialize the HS300x device structure */
  priv = (FAR struct hs300x_dev_s *)kmm_malloc(sizeof(struct hs300x_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;

  /* Register the character driver */
  ret = register_driver(devpath, &g_hs300xfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }
  
  sninfo("HS300X driver loaded successfully!\n");
  return ret;
}
#endif /* CONFIG_I2C && CONFIG_HS300X_I2C */
