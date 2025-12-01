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
 * @file qs-fs.c
 * @brief qs-fs wind speed driver base sensor
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.12.10
 */

#include <sensor.h>

static struct SensorDevice qs_fs;
static const unsigned char instructions[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};

static struct SensorProductInfo info =
{
    SENSOR_ABILITY_WINDSPEED,
    "清易电子",
    "QS-FS",
};

/**
 * @description: Open QS-FS voice device
 * @param sdev - sensor device pointer
 * @return success: 1 , failure: other
 */
#ifdef ADD_NUTTX_FETURES
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    sdev->fd = PrivOpen(SENSOR_DEVICE_QS_FS_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_QS_FS_DEV);
        return -1;
    }
    return sdev->fd;
}
#else
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    int result = 0;

    sdev->fd = PrivOpen(SENSOR_DEVICE_QS_FS_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_QS_FS_DEV);
        return -1;
    }
    
    struct SerialDataCfg cfg;
    cfg.serial_baud_rate    = BAUD_RATE_9600;
    cfg.serial_data_bits    = DATA_BITS_8;
    cfg.serial_stop_bits    = STOP_BITS_1;
    cfg.serial_buffer_size  = 64;
    cfg.serial_parity_mode  = PARITY_NONE;
    cfg.serial_bit_order    = 0;
    cfg.serial_invert_mode  = 0;
#ifdef SENSOR_QS_FS_DRIVER_EXTUART    
    cfg.ext_uart_no         = SENSOR_DEVICE_QS_FS_DEV_EXT_PORT;
    cfg.port_configure      = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &cfg;
    result = PrivIoctl(sdev->fd, OPE_INT, &ioctl_cfg);

    return result;
}
#endif

/**
 * @description: Read sensor device
 * @param sdev - sensor device pointer
 * @param len - the length of the read data
 * @return get data length
 */
static int SensorDeviceRead(struct SensorDevice *sdev, size_t len)
{
    if (PrivWrite(sdev->fd, instructions, sizeof(instructions)) < 0)
        return -1;
    
    if (PrivRead(sdev->fd, sdev->buffer, len) < 0)
        return -1;

    return 0;
}

static struct SensorDone done =
{
    SensorDeviceOpen,
    NULL,
    SensorDeviceRead,
    NULL,
    NULL,
};

/**
 * @description: Init QS-FS sensor and register
 * @return void
 */
static void QsFsInit(void)
{
    qs_fs.name = SENSOR_DEVICE_QS_FS;
    qs_fs.info = &info;
    qs_fs.done = &done;
    qs_fs.status = SENSOR_DEVICE_PASSIVE;

    SensorDeviceRegister(&qs_fs);
}


static struct SensorQuantity qs_fs_wind_speed;

/**
 * @description: Analysis QS-FS wind speed
 * @param quant - sensor quantity pointer
 * @return quantity value
 */
static int32_t ReadWindSpeed(struct SensorQuantity *quant)
{
    if (!quant)
        return -1;

    short result;
    if (quant->sdev->done->read != NULL) {
        if (quant->sdev->status == SENSOR_DEVICE_PASSIVE) {
            quant->sdev->done->read(quant->sdev, 6);
            result = (quant->sdev->buffer[3] << 8) | quant->sdev->buffer[4];

            return (int32_t)result;
        }
        if (quant->sdev->status == SENSOR_DEVICE_ACTIVE) {
            printf("Please set passive mode.\n");
        }
    }else{
        printf("%s don't have read done.\n", quant->name);
    }
    
    return -1;
}

/**
 * @description: Init QS-FS voice quantity and register
 * @return 0
 */
int QsFsWindSpeedInit(void)
{
    QsFsInit();
    
    qs_fs_wind_speed.name = SENSOR_QUANTITY_QS_FS_WINDSPEED;
    qs_fs_wind_speed.type = SENSOR_QUANTITY_WINDSPEED;
    qs_fs_wind_speed.value.decimal_places = 1;
    qs_fs_wind_speed.value.max_std = 600;
    qs_fs_wind_speed.value.min_std = 0;
    qs_fs_wind_speed.value.last_value = SENSOR_QUANTITY_VALUE_ERROR;
    qs_fs_wind_speed.value.max_value = SENSOR_QUANTITY_VALUE_ERROR;
    qs_fs_wind_speed.value.min_value = SENSOR_QUANTITY_VALUE_ERROR;
    qs_fs_wind_speed.sdev = &qs_fs;
    qs_fs_wind_speed.ReadValue = ReadWindSpeed;

    SensorQuantityRegister(&qs_fs_wind_speed);

    return 0;
}
