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
 * @file transform.h
 * @brief Interface function declarations required by the framework
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.06.04
 */

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <nuttx/config.h>
#include <nuttx/pthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/time.h>
#include <stddef.h>
#include <stdint.h>
#include <nuttx/wireless/lpwan/sx127x.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

#ifdef __cplusplus
extern "C" {
#endif

#define OPE_INT                  0x0000
#define OPE_CFG                 0x0001

#define NAME_NUM_MAX            32

/*********************GPIO define*********************/
#define GPIO_LOW    0x00
#define GPIO_HIGH   0x01

#define GPIO_CFG_OUTPUT                          0x00
#define GPIO_CFG_INPUT                               0x01
#define GPIO_CFG_INPUT_PULLUP            0x02
#define GPIO_CFG_INPUT_PULLDOWN     0x03
#define GPIO_CFG_OUTPUT_OD                  0x04

#define GPIO_CONFIG_MODE                 0xffffffff

/********************SERIAL define*******************/
#define BAUD_RATE_2400          2400
#define BAUD_RATE_4800          4800
#define BAUD_RATE_9600          9600
#define BAUD_RATE_19200         19200
#define BAUD_RATE_38400         38400
#define BAUD_RATE_57600         57600
#define BAUD_RATE_115200        115200
#define BAUD_RATE_230400        230400
#define BAUD_RATE_460800        460800
#define BAUD_RATE_921600        921600
#define BAUD_RATE_2000000       2000000
#define BAUD_RATE_3000000       3000000

#define DATA_BITS_5             5
#define DATA_BITS_6             6
#define DATA_BITS_7             7
#define DATA_BITS_8             8
#define DATA_BITS_9             9

#define STOP_BITS_1             1
#define STOP_BITS_2             2
#define STOP_BITS_3             3
#define STOP_BITS_4             4

#define PARITY_NONE             1
#define PARITY_ODD              2
#define PARITY_EVEN             3

#define BIT_ORDER_LSB           1
#define BIT_ORDER_MSB           2

#define NRZ_NORMAL              1
#define NRZ_INVERTED            2

#ifndef SERIAL_RB_BUFSZ
#define SERIAL_RB_BUFSZ         128
#endif

struct PinDevIrq
{
    int irq_mode;//< RISING/FALLING/HIGH/LOW
    void (*hdr) (void *args);//< callback function
    void *args;//< the params of callback function
};

struct PinParam
{
    int cmd;//< cmd:GPIO_CONFIG_MODE/GPIO_IRQ_REGISTER/GPIO_IRQ_FREE/GPIO_IRQ_DISABLE/GPIO_IRQ_ENABLE
    long  pin;//< pin number
    int mode;//< pin mode: input/output
    struct PinDevIrq irq_set;//< pin irq set
    uint64 arg;
};

struct PinStat
{
    long pin;//< pin number
    uint16_t val;//< pin level
};

enum ExtSerialPortConfigure
{
    PORT_CFG_INIT = 0,
    PORT_CFG_PARITY_CHECK,
    PORT_CFG_DISABLE,
    PORT_CFG_DIV,
};

struct SerialDataCfg
{
    uint32_t serial_baud_rate;
    uint8_t serial_data_bits;
    uint8_t serial_stop_bits;
    uint8_t serial_parity_mode;
    uint8_t serial_bit_order;
    uint8_t serial_invert_mode;
    uint16_t serial_buffer_size;

    uint8 ext_uart_no;
    enum ExtSerialPortConfigure port_configure;
};

enum IoctlDriverType
{
    SERIAL_TYPE = 0,
    SPI_TYPE,
    I2C_TYPE,
    PIN_TYPE,
    DEFAULT_TYPE,
};

struct PrivIoctlCfg
{
    enum IoctlDriverType ioctl_driver_type;
    void *args;
};

int open(FAR const char *path, int oflags, ...);
int ioctl(int fd, int req, ...);
FAR void *malloc(size_t size);
FAR void *realloc(FAR void *oldmem, size_t size);
FAR void *calloc(size_t n, size_t elem_size);
void free(FAR void *mem);

/**********************mutex**************************/

int PrivMutexCreate(pthread_mutex_t *p_mutex, const pthread_mutexattr_t *attr);
int PrivMutexDelete(pthread_mutex_t *p_mutex);
int PrivMutexObtain(pthread_mutex_t *p_mutex);
int PrivMutexAbandon(pthread_mutex_t *p_mutex);




/*********************semaphore**********************/

int PrivSemaphoreCreate(sem_t *sem, int pshared, unsigned int value);
int PrivSemaphoreDelete(sem_t *sem);
int PrivSemaphoreObtainWait(sem_t *sem, const struct timespec *abstime);
int PrivSemaphoreObtainWaitForever(sem_t *sem);
int PrivSemaphoreObtainNoWait(sem_t *sem);
int PrivSemaphoreAbandon(sem_t *sem);
int32_t PrivSemaphoreSetValue(int32_t sem, uint16_t val);

/*********************task**************************/

int PrivTaskCreate(pthread_t *thread, const pthread_attr_t *attr,
                   void *(*start_routine)(void *), void *arg);
int PrivTaskStartup(pthread_t *thread);
int PrivTaskDelete(pthread_t thread, int sig);
void PrivTaskQuit(void *value_ptr);
int PrivTaskDelay(int32_t ms);
uint32_t PrivGetTickTime(void);

/*********************driver*************************/

int PrivOpen(const char *path, int flags);
int PrivRead(int fd, void *buf, size_t len);
int PrivWrite(int fd, const void *buf, size_t len);
int PrivClose(int fd);
int PrivIoctl(int fd, int cmd, void *args);

/*********************memory***********************/

void *PrivMalloc(size_t size);
void *PrivRealloc(void *pointer, size_t size);
void *PrivCalloc(size_t  count, size_t size);
void PrivFree(void *pointer);


#ifdef __cplusplus
}
#endif

#endif