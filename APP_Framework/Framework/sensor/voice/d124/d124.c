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
 * @file d124.c
 * @brief D124 voice driver base sensor
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.04.22
 */

#include <sensor.h>

static struct SensorDevice d124;
static pthread_t active_task_id;
static pthread_mutex_t buff_lock;

static struct SensorProductInfo info =
{
    SENSOR_ABILITY_VOICE,
    "龙戈电子",
    "D124",
};

/**
 * @description: Read sensor task
 * @param sdev - sensor device pointer
 */
static void *ReadTask(void *parameter)
{
    struct SensorDevice *sdev = (struct SensorDevice *)parameter;
    while (1) {
        PrivMutexObtain(&buff_lock);
        sdev->done->read(sdev, 5);
        PrivMutexAbandon(&buff_lock);
        PrivTaskDelay(750);
    }
}

/**
 * @description: Open D124 voice device
 * @param sdev - sensor device pointer
 * @return success: 1 , failure: other
 */
#ifdef ADD_NUTTX_FETURES
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    int result = 0;
    pthread_attr_t attr = PTHREAD_ATTR_INITIALIZER;

    result = PrivMutexCreate(&buff_lock, NULL);
    if (result != 0){
      printf("SensorDeviceOpen:mutex create failed, status=%d\n", result);
    }

    sdev->fd = PrivOpen(SENSOR_DEVICE_D124_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("SensorDeviceOpen:open %s error\n", SENSOR_DEVICE_D124_DEV);
        return -1;
    }

    attr.priority = 20;
    attr.stacksize = 2048;

    result = PrivTaskCreate(&active_task_id, &attr, &ReadTask, sdev);
    if (result != 0){
      printf("SensorDeviceOpen:task create failed, status=%d\n", result);
    }
    PrivTaskStartup(&active_task_id);

    return result;
}
#else
static int SensorDeviceOpen(struct SensorDevice *sdev)
{
    int result = 0;
    pthread_attr_t attr;

    PrivMutexCreate(&buff_lock, 0);

    sdev->fd = PrivOpen(SENSOR_DEVICE_D124_DEV, O_RDWR);
    if (sdev->fd < 0) {
        printf("open %s error\n", SENSOR_DEVICE_D124_DEV);
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
#ifdef SENSOR_D124_DRIVER_EXTUART    
    cfg.ext_uart_no         = SENSOR_DEVICE_D124_DEV_EXT_PORT;
    cfg.port_configure      = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &cfg;
    result = PrivIoctl(sdev->fd, OPE_INT, &ioctl_cfg);

    attr.schedparam.sched_priority = 20;
    attr.stacksize = 2048;

    PrivTaskCreate(&active_task_id, &attr, &ReadTask, sdev);
    PrivTaskStartup(&active_task_id);

    return result;
}
#endif

/**
 * @description: Close D124 sensor device
 * @param sdev - sensor device pointer
 * @return 1
 */
static int SensorDeviceClose(struct SensorDevice *sdev)
{
    PrivTaskDelete(active_task_id, 0);
    PrivMutexDelete(&buff_lock);
    return 0;
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
    int timeout = 0;
    while (1) {
        PrivRead(sdev->fd, &tmp, 1);
        if ((tmp == 0xAA) || (timeout >= 1000))
            break;
        PrivTaskDelay(10);
        ++timeout;
    }
    
    if(tmp != 0xAA)
        return -1;

    uint8_t idx = 0;
    sdev->buffer[idx++] = tmp;

    while ((idx < len) && (timeout < 1000)) {
        if (PrivRead(sdev->fd, &tmp, 1) == 1) {
            timeout = 0;
            sdev->buffer[idx++] = tmp;
        }
        ++timeout;
    }

    return idx;
}

static struct SensorDone done =
{
    SensorDeviceOpen,
    SensorDeviceClose,
    SensorDeviceRead,
    NULL,
    NULL,
};

/**
 * @description: Init D124 sensor and register
 * @return void
 */
static void D124Init(void)
{
    d124.name = SENSOR_DEVICE_D124;
    d124.info = &info;
    d124.done = &done;
    d124.status = SENSOR_DEVICE_ACTIVE;

    SensorDeviceRegister(&d124);
}


static struct SensorQuantity d124_voice;

/**
 * @description: Analysis D124 voice result
 * @param quant - sensor quantity pointer
 * @return quantity value
 */
static int32_t ReadVoice(struct SensorQuantity *quant)
{
    if (!quant)
        return -1;

    uint32_t result;
    if (quant->sdev->done->read != NULL) {
        PrivMutexObtain(&buff_lock);      
        
        if (quant->sdev->buffer[3] == quant->sdev->buffer[1] + quant->sdev->buffer[2]) {
            result = ((uint16_t)quant->sdev->buffer[1] << 8) + (uint16_t)quant->sdev->buffer[2];
            if (result > quant->value.max_value)
                quant->value.max_value = result;
            else if (result < quant->value.min_value)
                quant->value.min_value = result;
            quant->value.last_value = result;

            return result;
        }else{
            printf("This reading is wrong\n");
            result = SENSOR_QUANTITY_VALUE_ERROR;
            return result;
        }
    }else{
        printf("%s don't have read done.\n", quant->name);
    }
    
    return -1;
}

/**
 * @description: Init D124 voice quantity and register
 * @return 0
 */
int D124VoiceInit(void)
{
    D124Init();
    
    d124_voice.name = SENSOR_QUANTITY_D124_VOICE;
    d124_voice.type = SENSOR_QUANTITY_VOICE;
    d124_voice.value.decimal_places = 1;
    d124_voice.value.max_std = 600;
    d124_voice.value.min_std = 0;
    d124_voice.value.last_value = SENSOR_QUANTITY_VALUE_ERROR;
    d124_voice.value.max_value = SENSOR_QUANTITY_VALUE_ERROR;
    d124_voice.value.min_value = SENSOR_QUANTITY_VALUE_ERROR;
    d124_voice.sdev = &d124;
    d124_voice.ReadValue = ReadVoice;

    SensorQuantityRegister(&d124_voice);

    return 0;
}
