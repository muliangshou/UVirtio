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
 * @file bmp180.c
 * @brief BMP180 altitude driver base sensor
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.12.20
 */

#include <sensor.h>
#include <math.h>

static struct SensorDevice bmp180;

typedef struct {
    int8_t ac_data[3];
    uint8_t unsigned_ac_data[3];
    int8_t b_data[2];
    int8_t m_data[3];
}Bmp180RegData;

static Bmp180RegData bmp180_reg_data;

const static unsigned char OSS = 0;  // Oversampling Setting
static long CalTemp_data = 0;

static struct SensorProductInfo info =
{
    (SENSOR_ABILITY_ALTITUDE),
    "BOSCH",
    "BMP180",
};

/**
 * @description: Open BMP180 sensor device
 * @param sdev - sensor device pointer
 * @return success : 0 error : -1
 */
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    int result;
    uint16_t i2c_dev_addr = SENSOR_DEVICE_BMP180_I2C_ADDR;
    
    sdev->fd = PrivOpen(SENSOR_DEVICE_BMP180_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_BMP180_DEV);
        return -1;
    }

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = I2C_TYPE;
    ioctl_cfg.args = &i2c_dev_addr;
    result = PrivIoctl(sdev->fd, OPE_INT, &ioctl_cfg);

    return result;
}

/**
 * @description: Write sensor device
 * @param sdev - sensor device pointer
 * @param len - the length of the read data
 * @return success: 0 , failure: -1
 */
static int SensorDeviceWrite(struct SensorDevice *sdev, const void *buf, size_t len)
{
    if (PrivWrite(sdev->fd, buf, len) < 0)
        return -1;

    return 0;
}

/**
 * @description: Read sensor device
 * @param sdev - sensor device pointer
 * @param len - the length of the read data
 * @return success: 0 , failure: -1
 */
static int SensorDeviceRead(struct SensorDevice *sdev, size_t len)
{
    //Read i2c device data from i2c device address
    if (PrivRead(sdev->fd, sdev->buffer, len) < 0)
        return -1;

    return 0;
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
 * @description: Init BMP180 sensor and register
 * @return void
 */
static void SensorDeviceBmp180Init(void)
{
    bmp180.name = SENSOR_DEVICE_BMP180;
    bmp180.info = &info;
    bmp180.done = &done;
    bmp180.status = SENSOR_DEVICE_PASSIVE;

    SensorDeviceRegister(&bmp180);
}

static struct SensorQuantity bmp180_altitude;

/**
 * @description: Stores all of the bmp180's calibration values into global variables
 * @return uint16_t reg data
 */
static uint16_t SensorDeviceBmp180ReadRegs(struct SensorDevice *sdev, char reg_addr)
{
    uint8_t reg_data[2] = {0};
    
    if (SensorDeviceWrite(sdev, &reg_addr, 1) < 0) {
        printf("SensorDeviceBmp180ReadRegs write reg 0x%x error. return 0x0\n", reg_addr);
        return 0;
    }
    
    if (SensorDeviceRead(sdev, 2) < 0) {
        printf("SensorDeviceBmp180ReadRegs error. return 0x0\n");
        return 0;
    }

    reg_data[0] = sdev->buffer[0];
    reg_data[1] = sdev->buffer[1];

    return (uint16_t)(reg_data[0] << 8 | reg_data[1]);
}

/**
 * @description: Read the uncompensated temperature value(ut)
 * @return uint16_t 0xF6 reg data
 */
static uint16_t SensorDeviceBmp180ReadUT(struct SensorDevice *sdev)
{
    uint8_t data[2];
    data[0] = 0xF4;
    data[1] = 0x2E;

    if (SensorDeviceWrite(sdev, data, 2) < 0) {
        printf("SensorDeviceBmp180ReadUT write reg 0xF4 error. return 0x0\n");
        return 0;
    }

    return SensorDeviceBmp180ReadRegs(sdev, 0xF6);
}

/**
 * @description: Read the uncompensated pressure value(up)
 * @return uint16_t 0xF6 reg data
 */
static uint16_t SensorDeviceBmp180ReadUP(struct SensorDevice *sdev)
{
    uint8_t data[2];
    data[0] = 0xF4;
    data[1] = 0x34 + (OSS << 6);

    if (SensorDeviceWrite(sdev, data, 2) < 0) {
        printf("SensorDeviceBmp180ReadUP write reg 0xF4 error. return 0x0\n");
        return 0;
    }

    return SensorDeviceBmp180ReadRegs(sdev, 0xF6);
}

/**
 * @description: Calculate temperature
 * @return uint16_t reg data
 */
static float SensorDeviceBmp180CalTemp(struct SensorDevice *sdev, uint16_t ut, Bmp180RegData *bmp180_reg_data)
{
    long x1, x2;
    x1 = (((long)ut - (long)bmp180_reg_data->unsigned_ac_data[2]) * (long)bmp180_reg_data->unsigned_ac_data[1]) >> 15;
    x2 = ((long)bmp180_reg_data->m_data[1] << 11) / (x1 + bmp180_reg_data->m_data[2]);
    CalTemp_data = x1 + x2;
    float temp = ((CalTemp_data + 8) >> 4);
    temp = temp / 10;
    return temp;
}

/**
 * @description: Calculate pressure
 * @return float pressure
 */
static float SensorDeviceBmp180CalPressure(struct SensorDevice *sdev, uint16_t up, Bmp180RegData *bmp180_reg_data)
{
    long x1, x2, x3, y1, y2, pressure;
    unsigned long z1, z2;

    y2 = CalTemp_data - 4000;
    // Calculate y1 data
    x1 = (bmp180_reg_data->b_data[1] * (y2 * y2) >> 12) >> 11;
    x2 = (bmp180_reg_data->ac_data[1] * y2)>>11;
    x3 = x1 + x2;
    y1 = (((((long)bmp180_reg_data->ac_data[0]) * 4 + x3) << OSS) + 2) >> 2;

    // Calculate B4
    x1 = (bmp180_reg_data->ac_data[2] * y2) >> 13;
    x2 = (bmp180_reg_data->b_data[0] * ((y2 * y2) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    z1 = (bmp180_reg_data->unsigned_ac_data[0] * (unsigned long)(x3 + 32768)) >> 15;

    z2 = ((unsigned long)(up - y1) * (50000 >> OSS));
    if (z2 < 0x80000000)
        pressure = (z2 << 1) / z1;
    else
        pressure = (z2 / z1) << 1;

    x1 = (pressure >> 8) * (pressure >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pressure) >> 16;
    pressure += (x1 + x2 + 3791) >> 4;

    return pressure;
}

/**
 * @description: Calculate altitude
 * @return float Altitude
 */
static float SensorDeviceBmp180CalAltitude(float pressure)
{
    float A = pressure / 101325;
    float B = 1 / 5.25588;
    float altitude = pow(A, B);
    altitude = 1 - altitude;
    altitude = altitude / 0.0000225577;

    return altitude;
}

/**
 * @description: Analysis BMP180 temperature result
 * @param quant - sensor quantity pointer
 * @return quantity value
 */
static int32_t ReadAltitude(struct SensorQuantity *quant)
{
    if (!quant)
        return -1;

    memset(&bmp180_reg_data, 0 , sizeof(Bmp180RegData));

    uint16_t ut, up = 0;
    float bmp180_temperature, bmp180_pressure, bmp180_aititude;
    if (quant->sdev->done->read != NULL) {
        if (quant->sdev->status == SENSOR_DEVICE_PASSIVE) {

            bmp180_reg_data.ac_data[0] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xAA);
            bmp180_reg_data.ac_data[1] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xAC);
            bmp180_reg_data.ac_data[2] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xAE);
            bmp180_reg_data.unsigned_ac_data[0] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xB0);
            bmp180_reg_data.unsigned_ac_data[1] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xB2);
            bmp180_reg_data.unsigned_ac_data[2] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xB4);
            bmp180_reg_data.b_data[0] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xB6);
            bmp180_reg_data.b_data[1] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xB8);
            bmp180_reg_data.m_data[0] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xBA);
            bmp180_reg_data.m_data[1] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xBC);
            bmp180_reg_data.m_data[2] = SensorDeviceBmp180ReadRegs(quant->sdev, 0xBE);

            ut = SensorDeviceBmp180ReadUT(quant->sdev);
            up = SensorDeviceBmp180ReadUP(quant->sdev);

            bmp180_temperature = SensorDeviceBmp180CalTemp(quant->sdev, ut, &bmp180_reg_data);
            bmp180_pressure = SensorDeviceBmp180CalPressure(quant->sdev, up, &bmp180_reg_data);
            bmp180_aititude = SensorDeviceBmp180CalAltitude(bmp180_pressure);

            return (int32_t)bmp180_pressure;
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
 * @description: Init BMP180 temperature quantity and register
 * @return 0
 */
int Bmp180AltitudeInit(void)
{
    SensorDeviceBmp180Init();
    
    bmp180_altitude.name = SENSOR_QUANTITY_BMP180_ALTITUDE;
    bmp180_altitude.type = SENSOR_QUANTITY_ALTITUDE;
    bmp180_altitude.value.decimal_places = 1;
    bmp180_altitude.value.max_std = 1000;
    bmp180_altitude.value.min_std = 0;
    bmp180_altitude.value.last_value = SENSOR_QUANTITY_VALUE_ERROR;
    bmp180_altitude.value.max_value = SENSOR_QUANTITY_VALUE_ERROR;
    bmp180_altitude.value.min_value = SENSOR_QUANTITY_VALUE_ERROR;
    bmp180_altitude.sdev = &bmp180;
    bmp180_altitude.ReadValue = ReadAltitude;

    SensorQuantityRegister(&bmp180_altitude);

    return 0;
}



