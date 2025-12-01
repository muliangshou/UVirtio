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
#include <stdio.h>
extern int FrameworkInit();
int main(void)
{
    printf("Hello World\n"); 
    FrameworkInit();
    return 0;
}
