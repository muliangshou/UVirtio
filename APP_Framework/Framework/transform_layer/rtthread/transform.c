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
 * @file xiuos.c
 * @brief Converts the framework interface to an operating system interface
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.06.07
 */

#include <transform.h>

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
    return sem_timedwait(sem, abstime);
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
    pthread_attr_t attrtmp ;    
    pthread_attr_init(&attrtmp);
    pthread_attr_setschedparam(&attrtmp, &(attr->schedparam)); /* 修改属性对应的优先级 */
    pthread_attr_setstacksize(&attrtmp, (size_t)((attr->stacksize)));
    return pthread_create(thread, &attrtmp, start_routine, arg);
}

int PrivTaskStartup(pthread_t *thread)
{
    return 0;
}

int PrivTaskDelete(pthread_t thread, int sig)
{
      _pthread_data_t *ptd;
    ptd = _pthread_get_data(thread);
    return rt_thread_detach(ptd->tid);
}

void PrivTaskQuit(void *value_ptr)
{
    pthread_exit(value_ptr);
}

int PrivTaskDelay(int32_t ms)
{
     rt_thread_mdelay(ms);
}

uint32_t PrivGetTickTime()
{
    return rt_tick_get_millisecond();
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
    struct dfs_fd *rt_fd;
    int ret = 0;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    struct SerialDataCfg *serial_cfg = (struct SerialDataCfg *)args;
    config.baud_rate = serial_cfg->serial_baud_rate;
    config.data_bits = serial_cfg->serial_data_bits;
    config.stop_bits = serial_cfg->serial_stop_bits;
    config.bufsz = RT_SERIAL_RB_BUFSZ;
    config.parity = serial_cfg->serial_parity_mode;
    config.invert = serial_cfg->serial_invert_mode;
    config.reserved = serial_cfg->ext_uart_no; ///< using extuart port number
    rt_fd = fd_get(fd);
    ret =  rt_fd->fops->ioctl(rt_fd, RT_DEVICE_CTRL_CONFIG, &config);
    return ret;
}

static int PrivPinIoctl(int fd, int cmd, void *args)
{
    struct PinParam *pin_cfg = (struct PinParam *)args;

    return ioctl(fd, cmd, pin_cfg);
}

int PrivIoctl(int fd, int cmd, void *args)
{
    int ret;
    struct PrivIoctlCfg *ioctl_cfg = (struct PrivIoctlCfg *)args;
    
    switch (ioctl_cfg->ioctl_driver_type)
    {
    case SERIAL_TYPE:
        ret = PrivSerialIoctl(fd, cmd, ioctl_cfg->args);
        break;
    case PIN_TYPE:
        ret = PrivPinIoctl(fd, cmd, ioctl_cfg->args);
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

