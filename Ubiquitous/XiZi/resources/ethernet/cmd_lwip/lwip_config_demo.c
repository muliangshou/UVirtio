/*
 * Copyright (c) 2021 AIIT XUOS Lab
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
 * @file lwip_config_demo.c
 * @brief Demo for ping function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.15
 */

#include "board.h"
#include "sys_arch.h"

/******************************************************************************/

static void LwipSetIPTask(void *param)
{
    lwip_config_net(lwip_ipaddr, lwip_netmask, lwip_gwaddr);
}

void LwipSetIPTest(int argc, char *argv[])
{
    if(argc >= 4)
    {
        lw_print("lw: [%s] ip %s mask %s gw %s\n", __func__, argv[1], argv[2], argv[3]);
        sscanf(argv[1], "%d.%d.%d.%d", &lwip_ipaddr[0], &lwip_ipaddr[1], &lwip_ipaddr[2], &lwip_ipaddr[3]);
        sscanf(argv[2], "%d.%d.%d.%d", &lwip_netmask[0], &lwip_netmask[1], &lwip_netmask[2], &lwip_netmask[3]);
        sscanf(argv[3], "%d.%d.%d.%d", &lwip_gwaddr[0], &lwip_gwaddr[1], &lwip_gwaddr[2], &lwip_gwaddr[3]);
    }
    else if(argc == 2)
    {
        lw_print("lw: [%s] ipaddr %s\n", __func__, argv[1]);
        sscanf(argv[1], "%d.%d.%d.%d", &lwip_ipaddr[0], &lwip_ipaddr[1], &lwip_ipaddr[2], &lwip_ipaddr[3]);
    }

    sys_thread_new("SET ip address", LwipSetIPTask, NULL, LWIP_TASK_STACK_SIZE, LWIP_DEMO_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     setip, LwipSetIPTest, SetIp [IP] [Netmask] [Gateway]);


void LwipShowIPTask(int argc, char *argv[])
{
    char mac_addr[] = configMAC_ADDR;

    lw_notice("\r\n************************************************\r\n");
    lw_notice(" Network Configuration\r\n");
    lw_notice("************************************************\r\n");
    lw_notice(" IPv4 Address   : %u.%u.%u.%u\r\n", ((u8_t *)&lwip_ipaddr)[0], ((u8_t *)&lwip_ipaddr)[1],
         ((u8_t *)&lwip_ipaddr)[2], ((u8_t *)&lwip_ipaddr)[3]);
    lw_notice(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&lwip_netmask)[0], ((u8_t *)&lwip_netmask)[1],
         ((u8_t *)&lwip_netmask)[2], ((u8_t *)&lwip_netmask)[3]);
    lw_notice(" IPv4 Gateway   : %u.%u.%u.%u\r\n", ((u8_t *)&lwip_gwaddr)[0], ((u8_t *)&lwip_gwaddr)[1],
         ((u8_t *)&lwip_gwaddr)[2], ((u8_t *)&lwip_gwaddr)[3]);
    lw_notice(" MAC Address    : %x:%x:%x:%x:%x:%x\r\n", mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);
    lw_notice("************************************************\r\n");
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
     showip, LwipShowIPTask, GetIp [IP] [Netmask] [Gateway]);

