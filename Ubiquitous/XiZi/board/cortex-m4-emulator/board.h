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
* @brief define cortex-m4-emulator-board init configure and start-up function
* @version 1.0 
* @author AIIT fudan Lab
* @date 2021-04-25
*/

/*************************************************
File name: board.h
Description: define cortex-m4-emulator-board board init function and struct
Others: 
History: 
1. Date: 2021-08-25
Author: AIIT fudan Lab
Modification: 
1. define cortex-m4-emulator-board InitBoardHardware
2. define cortex-m4-emulator-board data and bss struct
*************************************************/

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

extern int __stack_end__;
extern unsigned int g_service_table_start;
extern unsigned int g_service_table_end;

#define SURPORT_MPU

#define MEMORY_START_ADDRESS (&__stack_end__)
#define MEM_OFFSET         128
#define MEMORY_END_ADDRESS      (0x20000000 + MEM_OFFSET * 1024)


#ifdef SEPARATE_COMPILE
typedef int (*main_t)(int argc, char *argv[]);
typedef void (*exit_t)(void);
struct userspace_s
{
  main_t    us_entrypoint;
  exit_t    us_taskquit;
  uintptr_t us_textstart;
  uintptr_t us_textend;
  uintptr_t us_datasource;
  uintptr_t us_datastart;
  uintptr_t us_dataend;
  uintptr_t us_bssstart;
  uintptr_t us_bssend;
  uintptr_t us_heapend;
};
#define USERSPACE (( struct userspace_s *)(0x08080000))

#ifndef SERVICE_TABLE_ADDRESS
#define SERVICE_TABLE_ADDRESS    (0x20000000)
#endif

#define USER_SRAM_SIZE         64
#define USER_MEMORY_START_ADDRESS (USERSPACE->us_bssend)
#define USER_MEMORY_END_ADDRESS   (0x10000000 + USER_SRAM_SIZE * 1024)
#endif

void InitBoardHardware(void);

#endif