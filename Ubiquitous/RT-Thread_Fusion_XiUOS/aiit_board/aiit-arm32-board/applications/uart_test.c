/*
 * @Author: yongliang
 * @Date: 2022-03-31 16:00:00
 * @Description: uart_test code
 * @FilePath: \xiuos\Ubiquitous\RT_Thread\aiit_board\stm32f407_mini_board\applications\uart_test.c
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

int main (void)
{
    /*1.设置系统中断优先组*/

    /*2.串口初始化*/
    while(1)
        {
            if(USART_RX_STA&0X8000)
                {
                len =USART_RX_STA&0X3fff;
                printf("发送的消息为:\r\n");
                for(t=0;t<len;t++)
                    {
                    USART_SendData(USART1,USART_RX_BUF[T]);




}



