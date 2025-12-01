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
* @file sys_arch.c
* @brief In order to adapt to XiZi, some changes have been made to implement the LwIP interface.
* @version 1.0
* @author AIIT XUOS Lab
* @date 2021-05-29
*/
#include "debug.h"

#include <lwip/opt.h>
#include <lwip/arch.h>

#include "tcpip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/sio.h"
#include <lwip/stats.h>
#include <lwip/debug.h>
#include <lwip/sys.h>
#include "lwip/dhcp.h"
#include "tcpip_priv.h"

#if !NO_SYS
#include "sys_arch.h"
#endif

#include <string.h>
#include <xs_ktask.h>
#include <xs_ktick.h>
#include <xs_assign.h>
#include <xs_sem.h>
#include <xs_mutex.h>
#include <xs_ktask.h>
#include <xs_msg.h>

#include "board.h"
#include "ethernet.h"
#include "enet_ethernetif.h"
#include <transform.h>

char lwip_ipaddr[] = {192, 168, 250, 253};
char lwip_netmask[] = {255, 255, 255, 0};
char lwip_gwaddr[] = {192, 168, 250, 252};
char lwip_flag = 0;

x_ticks_t lwip_sys_now;

struct sys_timeouts {
  struct sys_timeo *next;
};

struct timeoutlist
{
  struct sys_timeouts timeouts;
  int32 pid;
};

#define SYS_THREAD_MAX 4

static struct timeoutlist s_timeoutlist[SYS_THREAD_MAX];

static u16_t s_nextthread = 0;

u32_t
sys_jiffies(void)
{
  lwip_sys_now = CurrentTicksGain();
  return lwip_sys_now;
}

u32_t
sys_now(void)
{
  lwip_sys_now = CurrentTicksGain();
  return lwip_sys_now;
}

void
sys_init(void)
{
  int i;
  for(i = 0; i < SYS_THREAD_MAX; i++)
  {
    s_timeoutlist[i].pid = 0;
    s_timeoutlist[i].timeouts.next = NULL;
  }
  s_nextthread = 0;
}

struct sys_timeouts *sys_arch_timeouts(void)
{
  int i;
  int32 pid;
  struct timeoutlist *tl;
  pid = (int32)GetKTaskDescriptor()->id.id;
  for(i = 0; i < s_nextthread; i++)
  {
    tl = &(s_timeoutlist[i]);
    if(tl->pid == pid)
    {
      return &(tl->timeouts);
    }
  }
  return NULL;
}

sys_prot_t sys_arch_protect(void)
{
  return CriticalAreaLock();
}

void sys_arch_unprotect(sys_prot_t pval)
{
  CriticalAreaUnLock(pval);
}

#if !NO_SYS

err_t
sys_sem_new(sys_sem_t *sem, u8_t count)
{
  *sem = KSemaphoreCreate((uint16)count);

#if SYS_STATS
  ++lwip_stats.sys.sem.used;
   if (lwip_stats.sys.sem.max < lwip_stats.sys.sem.used) {
    lwip_stats.sys.sem.max = lwip_stats.sys.sem.used;
  }
#endif /* SYS_STATS */

  if(*sem >= 0)
    return ERR_OK;
  else
  {
#if SYS_STATS
    ++lwip_stats.sys.sem.err;
#endif /* SYS_STATS */
    KPrintf("[sys_arch]:new sem fail!\n");
    return ERR_MEM;
  }
}

void
sys_sem_free(sys_sem_t *sem)
{
#if SYS_STATS
   --lwip_stats.sys.sem.used;
#endif /* SYS_STATS */
  KSemaphoreDelete(*sem);
  *sem = SYS_SEM_NULL;
}

int sys_sem_valid(sys_sem_t *sem)
{
  return (*sem > SYS_SEM_NULL);
}

void
sys_sem_set_invalid(sys_sem_t *sem)
{
  *sem = SYS_SEM_NULL;
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  x_ticks_t start_tick = 0 ;
  int32 wait_time = 0;


  if(*sem == SYS_SEM_NULL)
    return SYS_ARCH_TIMEOUT;


  start_tick = CurrentTicksGain();

  if (0 == timeout)
    wait_time = WAITING_FOREVER;
  else
    wait_time = timeout;


  if(KSemaphoreObtain(*sem, wait_time) == EOK)
    return ((CurrentTicksGain()-start_tick)*MS_PER_SYSTICK);
  else
    return SYS_ARCH_TIMEOUT;
}

void sys_sem_signal(sys_sem_t *sem)
{
  if(KSemaphoreAbandon( *sem ) != EOK)
    KPrintf("[sys_arch]:sem signal fail!\n");
}

err_t sys_mutex_new(sys_mutex_t *mutex)
{
  *mutex = KMutexCreate();
  if(*mutex > SYS_MRTEX_NULL)
    return ERR_OK;
  else
  {
    KPrintf("[sys_arch]:new mutex fail!\n");
    return ERR_MEM;
  }
}

void sys_mutex_free(sys_mutex_t *mutex)
{
  KMutexDelete(*mutex);
}

void sys_mutex_set_invalid(sys_mutex_t *mutex)
{
  *mutex = SYS_MRTEX_NULL;
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
  KMutexObtain(*mutex,
                 WAITING_FOREVER);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
  KMutexAbandon( *mutex );
}


sys_thread_t sys_thread_new(const char *name, lwip_thread_fn function, void *arg, int stacksize, int prio)
{
  sys_thread_t handle = -1;
  handle =  KTaskCreate(name,
                         function,
                         arg,
                         (uint32)stacksize,
                         (uint8)prio);
  if (handle >= 0)
  {
    StartupKTask(handle);
    lw_print("lw: [%s] create %s handle %x\n", __func__, name, handle);
    return handle;
  }
  lw_print("lw: [%s] create %s failed\n", __func__, name);
  return -ERROR;
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
  *mbox = KCreateMsgQueue(sizeof(void *), size);

#if SYS_STATS
      ++lwip_stats.sys.mbox.used;
      if (lwip_stats.sys.mbox.max < lwip_stats.sys.mbox.used) {
         lwip_stats.sys.mbox.max = lwip_stats.sys.mbox.used;
    }
#endif /* SYS_STATS */
  if(*mbox < 0)
  {
    lw_print("lw: [%s] alloc %d mbox %p failed\n", __func__, size, mbox);
    return ERR_MEM;
  }

  lw_print("lw: [%s] alloc %d mbox %p ok!\n", __func__, size, mbox);
  return ERR_OK;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
  KDeleteMsgQueue(*mbox);
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
  if (*mbox <= SYS_MBOX_NULL)
    return 0;
  else
    return 1;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  *mbox = SYS_MBOX_NULL;
}

void sys_mbox_post(sys_mbox_t *q, void *msg)
{
  while(KMsgQueueSendwait( *q, &msg, sizeof(void *), WAITING_FOREVER) != EOK);
}

err_t sys_mbox_trypost(sys_mbox_t *q, void *msg)
{
  if(KMsgQueueSend(*q, &msg, sizeof(void *)) == EOK)
    return ERR_OK;
  else
    return ERR_MEM;
}

err_t sys_mbox_trypost_fromisr(sys_mbox_t *q, void *msg)
{
  return sys_mbox_trypost(q, msg);
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *q, void **msg, u32_t timeout)
{
  x_ticks_t start_tick = 0 ;
  int32 wait_time = 0;

  start_tick = CurrentTicksGain();

  if (0 == timeout)
    wait_time = WAITING_FOREVER;
  else
    wait_time = timeout;

  if(KMsgQueueRecv(*q, &(*msg), sizeof(void *), wait_time) == EOK)
    return ((CurrentTicksGain()-start_tick)*MS_PER_SYSTICK);
  else{
    *msg = NULL;
    return SYS_ARCH_TIMEOUT;
  }
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *q, void **msg)
{
  if(KMsgQueueRecv(*q, &(*msg), sizeof(void *), 0) == EOK)
    return ERR_OK;
  else
    return SYS_MBOX_EMPTY;
}

#if LWIP_NETCONN_SEM_PER_THREAD
#error LWIP_NETCONN_SEM_PER_THREAD==1 not supported
#endif /* LWIP_NETCONN_SEM_PER_THREAD */

#endif /* !NO_SYS */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;

void lwip_tcp_init(void)
{
  tcpip_init(NULL, NULL);

  /* IP addresses initialization */
  /* USER CODE BEGIN 0 */
#if LWIP_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP4_ADDR(&ipaddr, lwip_ipaddr[0], lwip_ipaddr[1], lwip_ipaddr[2], lwip_ipaddr[3]);
  IP4_ADDR(&netmask, lwip_netmask[0], lwip_netmask[1], lwip_netmask[2], lwip_netmask[3]);
  IP4_ADDR(&gw, lwip_gwaddr[0], lwip_gwaddr[1], lwip_gwaddr[2], lwip_gwaddr[3]);
#endif /* USE_DHCP */
  /* USER CODE END 0 */
  /* Initilialize the LwIP stack without RTOS */
  /* add the network interface (IPv4/IPv6) without RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif0_init, &tcpip_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    KPrintf("%s : netif_set_up\n", __func__);
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    KPrintf("%s : netif_set_down\n", __func__);
    netif_set_down(&gnetif);
  }

#if LWIP_DHCP
  int err;
  /*  Creates a new DHCP client for this interface on the first call.
  Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
  the predefined regular intervals after starting the client.
  You can peek in the netif->dhcp struct for the actual DHCP status.*/


  err = dhcp_start(&gnetif);
  if(err == ERR_OK)
    KPrintf("lwip dhcp init success...\n\n");
  else
    KPrintf("lwip dhcp init fail...\n\n");
  while(ip_addr_cmp(&(gnetif.ip_addr),&ipaddr))
  {
    DelayKTask(1);
  }
#endif
  KPrintf("\n\nIP:%d.%d.%d.%d\n\n",  \
        ((gnetif.ip_addr.addr)&0x000000ff),       \
        (((gnetif.ip_addr.addr)&0x0000ff00)>>8),  \
        (((gnetif.ip_addr.addr)&0x00ff0000)>>16), \
        ((gnetif.ip_addr.addr)&0xff000000)>>24);
}

// lwip input thread to get network packet
void lwip_input_thread(void *param)
{
  struct netif *net = param;

  while (1)
  {
#ifdef FSL_RTOS_XIUOS
    if (lwip_obtain_semaphore(net) == EOK)
#endif
    {
        /* Poll the driver, get any outstanding frames */
        ethernetif_input(net);
        sys_check_timeouts(); /* Handle all system timeouts for all core protocols */
    }
  }
}

void lwip_config_input(struct netif *net)
{
  sys_thread_t th_id = 0;

  th_id = sys_thread_new("eth_input", lwip_input_thread, net, LWIP_TASK_STACK_SIZE, 15);

  if (th_id >= 0) {
    lw_print("%s %d successfully!\n", __func__, th_id);
  } else {
    lw_print("%s failed!\n", __func__);
  }
}

void lwip_config_net(char *ip, char *mask, char *gw)
{
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
  mem_range_t non_dma_memory[] = NON_DMA_MEMORY_ARRAY;
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
  ip4_addr_t net_ipaddr, net_netmask, net_gw;
  ethernetif_config_t cfg = {
    .phyAddress = BOARD_ENET0_PHY_ADDRESS,
    .clockName  = kCLOCK_CoreSysClk,
    .macAddress = configMAC_ADDR,
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
    .non_dma_memory = non_dma_memory,
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
  };

  ETH_BSP_Config();

  if(chk_lwip_bit(LWIP_INIT_FLAG))
  {
    lw_print("lw: [%s] already ...\n", __func__);

    IP4_ADDR(&net_ipaddr, ip[0], ip[1], ip[2], ip[3]);
    IP4_ADDR(&net_netmask, mask[0], mask[1], mask[2], mask[3]);
    IP4_ADDR(&net_gw, gw[0], gw[1], gw[2], gw[3]);

    // update ip addr
    netif_set_down(&gnetif);
    netif_set_gw(&gnetif, &net_gw);
    netif_set_netmask(&gnetif, &net_netmask);
    netif_set_ipaddr(&gnetif, &net_ipaddr);
    netif_set_up(&gnetif);
    return;
  }
  set_lwip_bit(LWIP_INIT_FLAG);

  lw_print("lw: [%s] start ...\n", __func__);

  IP4_ADDR(&net_ipaddr, ip[0], ip[1], ip[2], ip[3]);
  IP4_ADDR(&net_netmask, mask[0], mask[1], mask[2], mask[3]);
  IP4_ADDR(&net_gw, gw[0], gw[1], gw[2], gw[3]);

  lwip_init();

  netif_add(&gnetif, &net_ipaddr, &net_netmask, &net_gw, &cfg, ethernetif0_init,
        ethernet_input);
  netif_set_default(&gnetif);
  netif_set_up(&gnetif);

  if(chk_lwip_bit(LWIP_PRINT_FLAG))
  {
    lw_notice("\r\n************************************************\r\n");
    lw_notice(" Network Configuration\r\n");
    lw_notice("************************************************\r\n");
    lw_notice(" IPv4 Address   : %u.%u.%u.%u\r\n", ((u8_t *)&net_ipaddr)[0], ((u8_t *)&net_ipaddr)[1],
         ((u8_t *)&net_ipaddr)[2], ((u8_t *)&net_ipaddr)[3]);
    lw_notice(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&net_netmask)[0], ((u8_t *)&net_netmask)[1],
         ((u8_t *)&net_netmask)[2], ((u8_t *)&net_netmask)[3]);
    lw_notice(" IPv4 Gateway   : %u.%u.%u.%u\r\n", ((u8_t *)&net_gw)[0], ((u8_t *)&net_gw)[1],
         ((u8_t *)&net_gw)[2], ((u8_t *)&net_gw)[3]);
    lw_notice("************************************************\r\n");
  }
  lwip_config_input(&gnetif);
}

void lwip_config_tcp(char *ip, char *mask, char *gw)
{
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
  mem_range_t non_dma_memory[] = NON_DMA_MEMORY_ARRAY;
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
  ip4_addr_t net_ipaddr, net_netmask, net_gw;
  ethernetif_config_t cfg = {
    .phyAddress = BOARD_ENET0_PHY_ADDRESS,
    .clockName  = kCLOCK_CoreSysClk,
    .macAddress = configMAC_ADDR,
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
    .non_dma_memory = non_dma_memory,
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
  };

  ETH_BSP_Config();

  if(chk_lwip_bit(LWIP_INIT_FLAG))
  {
    lw_print("lw: [%s] already ...\n", __func__);
    return;
  }

  set_lwip_bit(LWIP_INIT_FLAG);

  tcpip_init(NULL, NULL);

  lw_print("lw: [%s] start ...\n", __func__);

  IP4_ADDR(&net_ipaddr, ip[0], ip[1], ip[2], ip[3]);
  IP4_ADDR(&net_netmask, mask[0], mask[1], mask[2], mask[3]);
  IP4_ADDR(&net_gw, gw[0], gw[1], gw[2], gw[3]);

  netif_add(&gnetif, &net_ipaddr, &net_netmask, &net_gw, &cfg, ethernetif0_init,
        tcpip_input);

  netif_set_default(&gnetif);
  netif_set_up(&gnetif);

  lw_print("\r\n************************************************\r\n");
  lw_print(" Network Configuration\r\n");
  lw_print("************************************************\r\n");
  lw_print(" IPv4 Address   : %u.%u.%u.%u\r\n", ((u8_t *)&net_ipaddr)[0], ((u8_t *)&net_ipaddr)[1],
       ((u8_t *)&net_ipaddr)[2], ((u8_t *)&net_ipaddr)[3]);
  lw_print(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&net_netmask)[0], ((u8_t *)&net_netmask)[1],
       ((u8_t *)&net_netmask)[2], ((u8_t *)&net_netmask)[3]);
  lw_print(" IPv4 Gateway   : %u.%u.%u.%u\r\n", ((u8_t *)&net_gw)[0], ((u8_t *)&net_gw)[1],
       ((u8_t *)&net_gw)[2], ((u8_t *)&net_gw)[3]);
  lw_print("************************************************\r\n");

  lwip_config_input(&gnetif);
}

