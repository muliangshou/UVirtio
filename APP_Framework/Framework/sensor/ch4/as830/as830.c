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
 * @file as830.c
 * @brief AS830 CH4 driver base sensor
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.10
 */

#include <sensor.h>

static struct SensorDevice as830;

static struct SensorProductInfo info =
{
    SENSOR_ABILITY_CH4,
    "xxxxx",
    "AS830",
};

/**
 * @description: Open AS830 sensor device
 * @param sdev - sensor device pointer
 * @return success: 1 , failure: other
 */
#ifdef ADD_NUTTX_FETURES
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    sdev->fd = PrivOpen(SENSOR_DEVICE_AS830_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_AS830_DEV);
        return -1;
    }

    return sdev->fd;
}
#else
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    int result = 0;

    sdev->fd = PrivOpen(SENSOR_DEVICE_AS830_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_AS830_DEV);
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
#ifdef SENSOR_AS830_DRIVER_EXTUART
    cfg.ext_uart_no         = SENSOR_DEVICE_AS830_DEV_EXT_PORT;
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
    uint8_t tmp = 0;
    uint8_t idx = 0;

    while (1) {
        if (PrivRead(sdev->fd, &tmp, 1) == 1) {
            sdev->buffer[idx++] = tmp;
        }
        if(0xA == tmp){
            break;
        }
    }

    return idx;
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
 * @description: Init AS830 sensor and register
 * @return void
 */
static void SensorDeviceAs830Init(void)
{
    as830.name = SENSOR_DEVICE_AS830;
    as830.info = &info;
    as830.done = &done;

    SensorDeviceRegister(&as830);
}


/* check data*/
static uint8_t CH4_check(uint8_t *packet)
{
    uint8_t i, high, low;
    uint8_t checksum = 0;
    for( i = 0; i < 10; i++)
    {
		if(packet[i] == 0x09)
			break;
        checksum += packet[i];
    }
    checksum = (unsigned char)(-(signed char)checksum);
	high = (checksum & 0xF0) >> 4;
	if((high <= 0x0F) && (high >= 0x0A))
		high += 0x37;
	else if((high <= 0x09) && (high >= 0x00))
		high += 0x30;
	low = (checksum & 0x0F);
	if((low <= 0x0F) && (low >= 0x0A))
		low += 0x37;
	else if((low <= 0x09) && (low >= 0x00))
		low += 0x30;
	if((high == packet[i + 1]) && (low == packet[i + 2]))
		return 0;
    
    return -1;
}


static struct SensorQuantity as830_ch4;

/**
 * @description: Analysis AS830 CH4 result
 * @param quant - sensor quantity pointer
 * @return quantity value
 */
static int32_t QuantityRead(struct SensorQuantity *quant)
{
    if (!quant)
        return -1;

    uint32_t result;
    uint32_t len = 0;
    if (quant->sdev->done->read != NULL) {
        if(quant->sdev->status == SENSOR_DEVICE_PASSIVE) {
            len = quant->sdev->done->read(quant->sdev, 7);
            if (len == 0)
            {
                printf("error read data length = 0.\n");
                return -1;
            }

            if(CH4_check(quant->sdev->buffer) == 0)
            {
                for(int i = 0; i < len; i++)
                {
                    if(quant->sdev->buffer[i] == 0x09)
                    {
                        break;
                    }
                    result = result *10 + (quant->sdev->buffer[i] - 0x30);
                }

                if (result > quant->value.max_value)
                    quant->value.max_value = result;
                else if (result < quant->value.min_value)
                    quant->value.min_value = result;
                quant->value.last_value = result;

                return result;
            }
            else
            {
                printf("This reading is wrong\n");
                result = SENSOR_QUANTITY_VALUE_ERROR;
                return result;
            }
        }
        if (quant->sdev->status == SENSOR_DEVICE_ACTIVE) 
        {
            printf("Please set passive mode.\n");
        }
    }else{
        printf("%s don't have read done.\n", quant->name);
    }
    
    return -1;
}

/**
 * @description: Init AS830 CH4 quantity and register
 * @return 0
 */
int As830Ch4Init(void)
{
    SensorDeviceAs830Init();
    
    as830_ch4.name = SENSOR_QUANTITY_AS830_CH4;
    as830_ch4.type = SENSOR_QUANTITY_CH4;
    as830_ch4.value.decimal_places = 0;
    as830_ch4.value.max_std = 1000;
    as830_ch4.value.min_std = 350;
    as830_ch4.value.last_value = SENSOR_QUANTITY_VALUE_ERROR;
    as830_ch4.value.max_value = SENSOR_QUANTITY_VALUE_ERROR;
    as830_ch4.value.min_value = SENSOR_QUANTITY_VALUE_ERROR;
    as830_ch4.sdev = &as830;
    as830_ch4.ReadValue = QuantityRead;

    SensorQuantityRegister(&as830_ch4);

    return 0;
}