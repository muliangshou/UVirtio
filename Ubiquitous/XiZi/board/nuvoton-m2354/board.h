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
* @brief define nuvoton-m2354 board init configure and start-up function
* @version 1.1 
* @author AIIT XUOS Lab
* @date 2022-02-24
*/

#ifndef __BOARD_H__
#define __BOARD_H__

#define SRAM_SIZE         (256)
#define SRAM_END          (0x20000000 + SRAM_SIZE * 1024)

extern int __bss_end;
#define HEAP_BEGIN      ((void *)&__bss_end)
#define HEAP_END        (void *)SRAM_END

void InitBoardHardware(void);

#endif
