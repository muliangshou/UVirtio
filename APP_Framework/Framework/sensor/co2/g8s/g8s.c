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
 * @file g8s.c
 * @brief G8S CO2 driver base sensor
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.12.23
 */

#include <sensor.h>
#include <math.h>

static uint8_t g8s_read_instruction[8] = {0x52, 0x36, 0x09, 0x37, 0x38, 0x0D, 0x0A};

static struct SensorDevice g8s;

static struct SensorProductInfo info =
{
    SENSOR_ABILITY_CO2,
    "senTec",
    "G8S",
};

/**
 * @description: Open G8S sensor device
 * @param sdev - sensor device pointer
 * @return success: 1 , failure: other
 */
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    int result = 0;

    sdev->fd = PrivOpen(SENSOR_DEVICE_G8S_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_G8S_DEV);
        return -1;
    }
    
    struct SerialDataCfg cfg;
    cfg.serial_baud_rate    = BAUD_RATE_9600;
    cfg.serial_data_bits    = DATA_BITS_8;
    cfg.serial_stop_bits    = STOP_BITS_1;
    cfg.serial_buffer_size  = 128;
    cfg.serial_parity_mode  = PARITY_NONE;
    cfg.serial_bit_order    = 0;
    cfg.serial_invert_mode  = 0;
#ifdef SENSOR_G8S_DRIVER_EXTUART
    cfg.ext_uart_no         = SENSOR_DEVICE_G8S_DEV_EXT_PORT;
    cfg.port_configure      = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &cfg;
    result = PrivIoctl(sdev->fd, OPE_INT, &ioctl_cfg);

    return result;
}

/**
 * @description: Read sensor device
 * @param sdev - sensor device pointer
 * @param len - the length of the read data
 * @return get data length
 */
static int SensorDeviceRead(struct SensorDevice *sdev, size_t len)
{
    uint8_t tmp = 0;

    PrivWrite(sdev->fd, g8s_read_instruction, sizeof(g8s_read_instruction));

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
 * @description: Init G8S sensor and register
 * @return void
 */
static void SensorDeviceG8sInit(void)
{
    g8s.name = SENSOR_DEVICE_G8S;
    g8s.info = &info;
    g8s.done = &done;
    g8s.status = SENSOR_DEVICE_PASSIVE;

    SensorDeviceRegister(&g8s);
}

static struct SensorQuantity g8s_co2;

/**
 * @description: Analysis G8S CO2 result
 * @param quant - sensor quantity pointer
 * @return quantity value
 */
static int32_t QuantityRead(struct SensorQuantity *quant)
{
    if (!quant)
        return -1;

    int i, ascii_length;
    int32_t result = 0;
    uint8_t result_ascii[9], result_hex[9];

    memset(result_ascii, 0, 9);
    memset(result_hex, 0, 9);

    if (quant->sdev->done->read != NULL) {
        if(quant->sdev->status == SENSOR_DEVICE_PASSIVE) {
            quant->sdev->done->read(quant->sdev, 9);

            for (i = 0; i < 9; i ++) {
                if (0x09 == quant->sdev->buffer[i]) {
                    ascii_length = i;
                    break;
                }
                result_ascii[i] = quant->sdev->buffer[i];
            }

            if (8 == ascii_length) {
                for (i = 0; i < ascii_length; i ++) {
                    result_hex[i] = result_ascii[i] - 0x30;
                    result += result_hex[i] * pow(10, ascii_length - 1 - i);
                }
                return result;
            } else {
                printf("This reading is wrong\n");
                result = SENSOR_QUANTITY_VALUE_ERROR;
                return result;
            }
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
 * @description: Init G8S CO2 quantity and register
 * @return 0
 */
int G8sCo2Init(void)
{
    SensorDeviceG8sInit();
    
    g8s_co2.name = SENSOR_QUANTITY_G8S_CO2;
    g8s_co2.type = SENSOR_QUANTITY_CO2;
    g8s_co2.value.decimal_places = 0;
    g8s_co2.value.max_std = 2000;
    g8s_co2.value.min_std = 0;
    g8s_co2.value.last_value = SENSOR_QUANTITY_VALUE_ERROR;
    g8s_co2.value.max_value = SENSOR_QUANTITY_VALUE_ERROR;
    g8s_co2.value.min_value = SENSOR_QUANTITY_VALUE_ERROR;
    g8s_co2.sdev = &g8s;
    g8s_co2.ReadValue = QuantityRead;

    SensorQuantityRegister(&g8s_co2);

    return 0;
}