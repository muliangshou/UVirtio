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
* @brief define rvstar-board init configure and start-up function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-09-02
*/

/*************************************************
File name: board.h
Description: define rvstar-board init configure and start-up function
Others: 
History: 
1. Date: 2021-09-02
Author: AIIT XUOS Lab
Modification: 
1. define rvstar-board InitBoardHardware
2. define rvstar-board data and bss struct
*************************************************/
#ifndef BOARD_H__
#define BOARD_H__

#include <xsconfig.h>
#include <stdint.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern unsigned int _end;
extern unsigned int __stack_end__;
extern unsigned int g_service_table_start;
extern unsigned int g_service_table_end;

#define MEMORY_START_ADDRESS  (void*)(&_end)
#define gd32vf103_SRAM_SIZE         0x00008000
#define MEMORY_END_ADDRESS     (void*)(0x20000000 + gd32vf103_SRAM_SIZE)


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif

#endif 
