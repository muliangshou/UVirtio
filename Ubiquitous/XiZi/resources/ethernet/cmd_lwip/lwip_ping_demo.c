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
 * @file lwip_ping_demo.c
 * @brief Demo for ping function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.15
 */

#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_RAW

#include "ping.h"

#include "lwip/timeouts.h"
#include "lwip/init.h"
#include "netif/ethernet.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

#include <sys_arch.h>
#include "connect_ethernet.h"

char test_ip_addr[] = {192, 168, 250, 253};
char test_net_mask[] = {255, 255, 255, 0};
char test_gw_addr[] = {192, 168, 250, 252};
ip4_addr_t ping_addr;

/******************************************************************************/

void LwipPingTest(int argc, char *argv[])
{
    int result = 0;

    if(argc >= 4)
    {
        lw_print("lw: [%s] ip %s mask %s gw %s\n", __func__, argv[1], argv[2], argv[3]);
        sscanf(argv[1], "%d.%d.%d.%d", &lwip_ipaddr[0], &lwip_ipaddr[1], &lwip_ipaddr[2], &lwip_ipaddr[3]);
        sscanf(argv[2], "%d.%d.%d.%d", &lwip_netmask[0], &lwip_netmask[1], &lwip_netmask[2], &lwip_netmask[3]);
        sscanf(argv[3], "%d.%d.%d.%d", &lwip_gwaddr[0], &lwip_gwaddr[1], &lwip_gwaddr[2], &lwip_gwaddr[3]);
    }
    else if(argc == 2)
    {
        lw_print("lw: [%s] gw %s\n", __func__, argv[1]);
        if(isdigit(argv[1][0]))
        {
            if(sscanf(argv[1], "%d.%d.%d.%d", &lwip_gwaddr[0], &lwip_gwaddr[1], &lwip_gwaddr[2], &lwip_gwaddr[3]) == EOF)
            {
                lw_notice("input wrong ip\n");
                return;
            }
        }
#if (LWIP_DHCP) && (PING_USE_SOCKETS)
        else
        {
            get_url_ip(argv[1]);
            return;
        }
#endif
    }

    lw_print("lw: [%s] argc %d\n", __func__, argc);

    IP4_ADDR(&ping_addr, lwip_gwaddr[0], lwip_gwaddr[1], lwip_gwaddr[2], lwip_gwaddr[3]);
    lwip_config_net(lwip_ipaddr, lwip_netmask, lwip_gwaddr);
    ping_init(&ping_addr);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     ping, LwipPingTest, ping [IP] 3 times);

#endif
