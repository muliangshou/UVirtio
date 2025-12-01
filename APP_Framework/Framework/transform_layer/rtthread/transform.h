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
#include <rtthread.h>
#include <rtdevice.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>
#include <errno.h>
#include <pthread.h>
#include <pthread_internal.h>
#include <semaphore.h>
#include <sched.h>
#include <unistd.h>
#include <dfs_poll.h>
#include <dfs_posix.h>
#include <dfs.h>
#include<sys/ioctl.h>
#ifdef RT_USING_POSIX_TERMIOS
#include <posix_termios.h>
#endif
#ifdef DRV_USING_OV2640
#include <drv_ov2640.h>
#endif
#if defined(RT_USING_SAL)&& defined (RT_USING_LWIP)
#include <netdb.h>
#include <sys/socket.h>
#elif defined RT_USING_LWIP
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#endif /* RT_USING_SAL */
#ifdef BOARD_K210_EVB
#include <dmac.h>
#include <dmalock.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif

typedef signed   char                   int8;      
typedef signed   short                  int16;    
typedef signed   int                    int32;     
typedef unsigned char                   uint8;    
typedef unsigned short                  uint16;    
typedef unsigned int                    uint32;    

#ifdef ARCH_CPU_64BIT
typedef signed long                     int64;    
typedef unsigned long                   uint64;    
#else
typedef signed long long                int64;     
typedef unsigned long long              uint64;    
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
    uint64_t arg;
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

    uint8_t ext_uart_no;
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

/**********************mutex**************************/

int PrivMutexCreate(pthread_mutex_t *p_mutex, const pthread_mutexattr_t *attr);
int PrivMutexDelete(pthread_mutex_t *p_mutex);
int PrivMutexObtain(pthread_mutex_t *p_mutex);
int PrivMutexAbandon(pthread_mutex_t *p_mutex);

/*********************semaphore**********************/

int PrivSemaphoreCreate(sem_t *sem, int pshared, unsigned int value);
int PrivSemaphoreDelete(sem_t *sem);
int PrivSemaphoreObtainWait(sem_t *sem, const struct timespec *abstime);
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
uint32_t PrivGetTickTime();

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