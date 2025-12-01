/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-04-29     tyustli      first version
 *
 */

#include <rtdevice.h>
#include <rtthread.h>
#include "drv_gpio.h"
#include <board.h>

/* defined the LED pin: GPIO1_IO9 */
#define LED0_PIN               GET_PIN(1,9)

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_kprintf("XIUOS xidatong build %s %s\n",__DATE__,__TIME__);
    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

#ifdef BSP_USING_SDRAM
static void sdram_test2(void)
{
    char *p =NULL;
	p = rt_malloc(1024*1024*8);
	if(p == NULL)
	{
		rt_kprintf("apply for 8MB memory fail ~!!!");
	}
	else
	{
		rt_kprintf("appyle for 8MB memory success!!!");
	}
	rt_free(p);
}
MSH_CMD_EXPORT(sdram_test2, sdram test2);
#endif
