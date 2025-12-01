/*
 * @Author: your name
 * @Date: 2021-10-11 22:04:25
 * @LastEditTime: 2021-10-14 11:12:52
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \xiuos\Ubiquitous\RT_Thread\bsp\k210\applications\main.c
 */
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

#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#define LED_G   12

int main(void)
{
    rt_pin_mode(LED_G, PIN_MODE_OUTPUT);
    rt_thread_mdelay(100);
    char info1[25] ={0};
    char info2[25] ={0};
    sprintf(info1,"xuos-intelligence k210  ");
    sprintf(info2,"build %s %s",__DATE__,__TIME__);
    printf("%s %s \n",info1,info2); 
    #ifdef BSP_USING_LCD
    #include<drv_lcd.h>
    lcd_show_string(0,60,24,info1);
    lcd_show_string(0,90,24,info2);
    #endif
    while(1)
    {
        rt_pin_write(LED_G, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_G, PIN_LOW);
        rt_thread_mdelay(500);
    }
    return 0;
}
