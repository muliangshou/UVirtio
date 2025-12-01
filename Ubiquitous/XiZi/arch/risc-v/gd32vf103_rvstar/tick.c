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
#include <xs_ktick.h>
#include <gd32vf103.h>
#include <core_feature_timer.h>

#define SysTick_Handler     eclic_mtip_handler

/* This is the timer interrupt service routine. */
void SysTick_Handler(void)
{
    SysTick_Reload(SYSTICK_TICK_CONST);

    // isrManager.done->incCounter();

    TickAndTaskTimesliceUpdate();

    // isrManager.done->decCounter();
}
