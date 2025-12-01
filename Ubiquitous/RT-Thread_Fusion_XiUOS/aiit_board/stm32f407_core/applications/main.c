/*
 * @Author: chunyexixiaoyu
 * @Date: 2021-09-24 16:33:15
 * @LastEditTime: 2021-09-24 15:48:30
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \xiuos\Ubiquitous\RT_Thread\bsp\stm32f407-atk-coreboard\applications\main.c
 */

/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtthread.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#ifdef RT_USING_POSIX
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <dfs_poll.h>
#include <dfs_posix.h>
#include <dfs.h>
#ifdef RT_USING_POSIX_TERMIOS
#include <posix_termios.h>
#endif
#endif

#define LED0_PIN    GET_PIN(F, 9)
extern int FrameworkInit();
int main(void)
{
     int count = 1;
     rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
     rt_thread_mdelay(100);
     FrameworkInit();
     printf("XIUOS stm32f4 build %s %s\n",__DATE__,__TIME__);          
     while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
