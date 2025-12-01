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
* @file board.h
* @brief define stm32f103-nano init configure and start-up function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-11-25
*/

#ifndef __BOARD_H__
#define __BOARD_H__

#include <stm32f1xx.h>
#include <connect_uart.h>

extern void *__bss_end;
extern void *_heap_end;

#define HEAP_START  ((void *)&__bss_end)
#define HEAP_END    ((void *)&_heap_end)

void SystemClock_Config(void);

#endif
