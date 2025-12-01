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
 * @file transform.c
 * @brief Converts the framework interface to an operating system interface
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.06.07
 */

#include "transform.h"

/**************************mutex***************************/
/* private mutex API */
int PrivMutexCreate(pthread_mutex_t *p_mutex, const pthread_mutexattr_t *attr)
{
    return pthread_mutex_init(p_mutex, attr);
}

int PrivMutexDelete(pthread_mutex_t *p_mutex)
{
    return pthread_mutex_destroy(p_mutex);
}

int PrivMutexObtain(pthread_mutex_t *p_mutex)
{
    return pthread_mutex_lock(p_mutex);
}

int PrivMutexAbandon(pthread_mutex_t *p_mutex)
{
    return pthread_mutex_unlock(p_mutex);
}

/**********************semaphore****************************/
int PrivSemaphoreCreate(sem_t *sem, int pshared, unsigned int value)
{
    return sem_init(sem, pshared, value);
}

int PrivSemaphoreDelete(sem_t *sem)
{
    return sem_destroy(sem);
}

int PrivSemaphoreObtainWait(sem_t *sem, const struct timespec *abstime)
{
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += abstime->tv_sec;
    return sem_timedwait(sem, &timeout);
}

int PrivSemaphoreObtainWaitForever(sem_t *sem)
{
    return sem_wait(sem);
}

int PrivSemaphoreObtainNoWait(sem_t *sem)
{
    return sem_trywait(sem);
}

int PrivSemaphoreAbandon(sem_t *sem)
{
    return sem_post(sem);
}

/**************************task*************************/
int PrivTaskCreate(pthread_t *thread, const pthread_attr_t *attr,
                   void *(*start_routine)(void *), void *arg)
{
    return pthread_create(thread, attr, start_routine, arg);
}

int PrivTaskStartup(pthread_t *thread)
{
    return 0;
}

int PrivTaskDelete(pthread_t thread, int sig)
{
    return pthread_kill(thread, sig);
}

void PrivTaskQuit(void *value_ptr)
{
    pthread_exit(value_ptr);
}

int PrivTaskDelay(int32_t ms)
{
    return usleep(1000 * ms);
}

uint32_t PrivGetTickTime(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/*********************fs**************************/

/************************Driver Posix Transform***********************/
int PrivOpen(const char *path, int flags)
{
    return open(path, flags);
}

int PrivClose(int fd)
{
    return close(fd);
}

int PrivRead(int fd, void *buf, size_t len)
{    
    return read(fd, buf, len);
}

int PrivWrite(int fd, const void *buf, size_t len)
{   
    return write(fd, buf, len);
}

static int PrivSerialIoctl(int fd, int cmd, void *args)
{
    struct SerialDataCfg *serial_cfg = (struct SerialDataCfg *)args;
    return ioctl(fd, cmd, serial_cfg);
}

static int PrivPinIoctl(int fd, int cmd, void *args)
{
    struct PinParam *pin_cfg = (struct PinParam *)args;

    return ioctl(fd, cmd, pin_cfg);
}

int PrivIoctl(int fd, int cmd, void *args)
{
    int ret = 0;
    struct PrivIoctlCfg *ioctl_cfg = (struct PrivIoctlCfg *)args;
    
    switch (ioctl_cfg->ioctl_driver_type)
    {
    case SERIAL_TYPE:
        ret = PrivSerialIoctl(fd, cmd, ioctl_cfg->args);
        break;
    case PIN_TYPE:
        ret = PrivPinIoctl(fd, cmd, ioctl_cfg->args);
        break;
    case I2C_TYPE:
        ret = ioctl(fd, cmd, ioctl_cfg->args);
        break;
    default:
        break;
    }

    return ret;
}

/********************memory api************/
void *PrivMalloc(size_t size)
{
    return malloc(size);
}

void *PrivRealloc(void *pointer, size_t size)
{
    return realloc(pointer, size);
}

void *PrivCalloc(size_t  count, size_t size)
{
    return calloc(count, size);
}

void PrivFree(void *pointer)
{
    free(pointer);
}

