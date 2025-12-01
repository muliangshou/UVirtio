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
 * @file tb600b_iaq10.c
 * @brief tb600b_iaq10 driver base sensor
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.14
 */

#include <sensor.h>

static struct SensorDevice tb600b_iaq10;
static uint8_t ReadInstruction[9];

struct iaq_data {
    uint16_t gas;
    uint8_t TH;
    uint8_t TL;
    uint8_t RhH;
    uint8_t RhL;
};

static struct SensorProductInfo info =
{
    SENSOR_ABILITY_IAQ,
    "AQS",
    "TB600B_IAQ10",
};

/**
 * @description: Open tb600b_iaq10 sensor device
 * @param sdev - sensor device pointer
 * @return success: 1 , failure: other
 */
#ifdef ADD_NUTTX_FETURES
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    sdev->fd = PrivOpen(SENSOR_DEVICE_TB600B_IAQ10_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_TB600B_IAQ10_DEV);
    }

    return sdev->fd;
}
#else
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    int result = 0;

    sdev->fd = PrivOpen(SENSOR_DEVICE_TB600B_IAQ10_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_TB600B_IAQ10_DEV);
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
#ifdef SENSOR_TB600B_IAQ10_DRIVER_EXTUART
    cfg.ext_uart_no         = SENSOR_DEVICE_TB600B_IAQ10_DEV_EXT_PORT;
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
    int32_t ret = 0;

    /* this instruction will read gas with temperature and humidity,return 13 datas*/
    ReadInstruction[0] = 0xFF;
    ReadInstruction[1] = 0x01;
    ReadInstruction[2] = 0x87;
    ReadInstruction[3] = 0x00;
    ReadInstruction[4] = 0x00;
    ReadInstruction[5] = 0x00;
    ReadInstruction[6] = 0x00;
    ReadInstruction[7] = 0x00;
    ReadInstruction[8] = 0x78;

    PrivWrite(sdev->fd, ReadInstruction, 9);

    for(idx = 0; idx < 13; idx++) 
    {
        if (PrivRead(sdev->fd, &tmp, 1) == 1) {
            sdev->buffer[idx] = tmp;
        }
    }

    return idx;
}
/**
 * @description: Read sensor device
 * @param sdev - sensor device pointer
 * @param len - the length of the read data
 * @return get data length
 */
static int SensorDeviceWrite(struct SensorDevice *sdev, const void *buf, size_t len)
{
    return PrivWrite(sdev->fd, buf, len);

}

static struct SensorDone done =
{
    SensorDeviceOpen,
    NULL,
    SensorDeviceRead,
    SensorDeviceWrite,
    NULL,
};

/**
 * @description: Init tb600b_iaq10 sensor and register
 * @return void
 */
static void SensorDeviceTb600bIaq10Init(void)
{
    tb600b_iaq10.name = SENSOR_DEVICE_TB600B_IAQ10;
    tb600b_iaq10.info = &info;
    tb600b_iaq10.done = &done;

    SensorDeviceRegister(&tb600b_iaq10);
}

static struct SensorQuantity tb600b_iaq10_iaq;

/* check data*/
static uint8_t getCheckSum(uint8_t *packet)
{
    uint8_t i;
    uint8_t checksum = 0;
    for( i = 1; i < 12; i++)
    {
        checksum += packet[i];
    }
    checksum = ~checksum + 1;
    
    return checksum;
}

/**
 * @description: Analysis tb600b_iaq10 result
 * @param quant - sensor quantity pointer
 * @return quantity value
 */
static int32_t QuantityRead(struct SensorQuantity *quant)
{
    if (!quant)
        return -1;

    uint32_t len = 0;
    uint8_t checksum = 0;
    uint8_t TH, TL, RhH, RhL;
    uint16_t gas;
    struct iaq_data result;

    if (quant->sdev->done->read != NULL) {
        if(quant->sdev->status == SENSOR_DEVICE_PASSIVE) {
            len = quant->sdev->done->read(quant->sdev, 13);
            if (len == 0)
            {
                printf("error read data length = 0.\n");
                return -1;
            }
            checksum = getCheckSum(quant->sdev->buffer);
            if(checksum == quant->sdev->buffer[12])
            {
                result.gas = (uint16_t)quant->sdev->buffer[6] * 256 + (uint16_t)quant->sdev->buffer[7];
                result.TH = ((int)((quant->sdev->buffer[8] << 8)|quant->sdev->buffer[9]))/100;
                result.TL = ((int)((quant->sdev->buffer[8] << 8)|quant->sdev->buffer[9]))%100;
                result.RhH = ((unsigned int)((quant->sdev->buffer[10] << 8)|quant->sdev->buffer[11]))/100;
                result.RhL = ((unsigned int)((quant->sdev->buffer[10] << 8)|quant->sdev->buffer[11]))%100;
                printf("Gas concentration is : %dppb\nThe temperature is : %d.%d℃\nThe humidity is : %d.%drh%%\n", result.gas, result.TH, result.TL, result.RhH, result.RhL);
                return result.gas;
            }
            else
            {
                printf("This reading is wrong\n");
                return SENSOR_QUANTITY_VALUE_ERROR;
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
 * @description: Init tb600b_iaq10 quantity and register
 * @return 0
 */
int Tb600bIaq10IaqInit(void)
{
    SensorDeviceTb600bIaq10Init();
    
    tb600b_iaq10_iaq.name = SENSOR_QUANTITY_TB600B_IAQ;
    tb600b_iaq10_iaq.type = SENSOR_QUANTITY_IAQ;
    tb600b_iaq10_iaq.value.decimal_places = 0;
    tb600b_iaq10_iaq.value.max_std = SENSOR_QUANTITY_VALUE_ERROR;
    tb600b_iaq10_iaq.value.min_std = SENSOR_QUANTITY_VALUE_ERROR;
    tb600b_iaq10_iaq.value.last_value = SENSOR_QUANTITY_VALUE_ERROR;
    tb600b_iaq10_iaq.value.max_value = SENSOR_QUANTITY_VALUE_ERROR;
    tb600b_iaq10_iaq.value.min_value = SENSOR_QUANTITY_VALUE_ERROR;
    tb600b_iaq10_iaq.sdev = &tb600b_iaq10;
    tb600b_iaq10_iaq.ReadValue = QuantityRead;

    SensorQuantityRegister(&tb600b_iaq10_iaq);

    return 0;
}