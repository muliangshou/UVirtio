/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-14     supperthomas first version
 * 2022-03-14     wwt          add xiuos framework
 */

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

#define LEDR_PIN    GET_PIN(C, 0)
extern int FrameworkInit();

int main(void)
{
    rt_pin_mode(LEDR_PIN, PIN_MODE_OUTPUT);
    FrameworkInit();
    printf("XIUOS stm32h7 build %s %s\n",__DATE__,__TIME__);
    #ifdef BSP_USING_USBD
    //rt_console_set_device("vcom");
    #endif
    while (1)
    {
        rt_pin_write(LEDR_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LEDR_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
