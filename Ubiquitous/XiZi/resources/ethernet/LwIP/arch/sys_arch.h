/*
 * Copyright (c) 2017 Simon Goldschmidt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt
 *
 */

/**
* @file sys_arch.h
* @brief In order to adapt to XiZi, some changes have been made to implement the LwIP interface.
* @version 1.0
* @author AIIT XUOS Lab
* @date 2021-05-29
*/

#ifndef __SYS_ARCH__
#define __SYS_ATCH__

#include <lwip/opt.h>
#include <lwip/arch.h>
#include <lwip/netif.h>

#include "tcpip.h"

#include <xs_base.h>

/* LWIP task parameter */
#define LWIP_LOCAL_PORT             4840
#define LWIP_TARGET_PORT            LWIP_LOCAL_PORT

#define LWIP_DEMO_TIMES             3
#define LWIP_TASK_STACK_SIZE        4096
#define LWIP_DEMO_TASK_PRIO         20

/* MAC address configuration. */
#define configMAC_ADDR {0x02, 0x12, 0x13, 0x10, 0x15, 0x11}

/* USER CODE END 0 */
#define SYS_MBOX_NULL  -1
#define SYS_SEM_NULL   -1
#define SYS_MRTEX_NULL SYS_SEM_NULL

typedef int32 sys_sem_t;
typedef int32 sys_mutex_t;
typedef int32 sys_mbox_t;
typedef int32 sys_thread_t;
typedef x_base sys_prot_t;

#define MS_PER_SYSTICK (1000 / TICK_PER_SECOND)

//debug rtos with IRQ
//#define FSL_RTOS_XIUOS

extern char lwip_flag;

#define LWIP_INIT_FLAG (1 << 0)
#define LWIP_PRINT_FLAG (1 << 1)

#define set_lwip_bit(__bit) lwip_flag |= (__bit)
#define clr_lwip_bit(__bit) lwip_flag &= ~(__bit)
#define chk_lwip_bit(__bit) ((lwip_flag & (__bit)) == (__bit))

extern char lwip_ipaddr[];
extern char lwip_netmask[];
extern char lwip_gwaddr[];
extern struct netif gnetif;

void lwip_tcp_init(void);
void lwip_config_net(char *ip, char *mask, char *gw);
void lwip_config_tcp(char *ip, char *mask, char *gw);

#endif
