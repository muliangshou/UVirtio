/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file lwip_dhcp_demo.c
 * @brief Demo for DHCP function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.15
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_DHCP

#include "lwip/timeouts.h"
#include "lwip/ip_addr.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"

#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "sys_arch.h"

#define LWIP_DHCP_TIME 10000 // 10s

/******************************************************************************/

/*!
 * @brief Prints DHCP status of the interface when it has changed from last status.
 *
 * @param netif network interface structure
 */
int LwipPrintDHCP(struct netif *netif)
{
    static u8_t dhcp_last_state = DHCP_STATE_OFF;
    struct dhcp *dhcp           = netif_dhcp_data(netif);

    if (dhcp == NULL)
    {
        dhcp_last_state = DHCP_STATE_OFF;
    }
    else if (dhcp_last_state != dhcp->state)
    {
        dhcp_last_state = dhcp->state;

        lw_notice(" DHCP state       : ");
        switch (dhcp_last_state)
        {
            case DHCP_STATE_OFF:
                lw_notice("OFF");
                break;
            case DHCP_STATE_REQUESTING:
                lw_notice("REQUESTING");
                break;
            case DHCP_STATE_INIT:
                lw_notice("INIT");
                break;
            case DHCP_STATE_REBOOTING:
                lw_notice("REBOOTING");
                break;
            case DHCP_STATE_REBINDING:
                lw_notice("REBINDING");
                break;
            case DHCP_STATE_RENEWING:
                lw_notice("RENEWING");
                break;
            case DHCP_STATE_SELECTING:
                lw_notice("SELECTING");
                break;
            case DHCP_STATE_INFORMING:
                lw_notice("INFORMING");
                break;
            case DHCP_STATE_CHECKING:
                lw_notice("CHECKING");
                break;
            case DHCP_STATE_BOUND:
                lw_notice("BOUND");
                break;
            case DHCP_STATE_BACKING_OFF:
                lw_notice("BACKING_OFF");
                break;
            default:
                lw_notice("%u", dhcp_last_state);
                assert(0);
                break;
        }
        lw_notice("\r\n");

        if (dhcp_last_state == DHCP_STATE_BOUND)
        {
            lw_notice("\r\n IPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
            lw_notice(" IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
            lw_notice(" IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));
            return 1;
        }
    }
    return 0;
}

/*!
 * @brief Main function.
 */
void LwipDHCPTest(void)
{
    u32_t dhcp_time;
    static int flag = 0;
    char ip_addr[4] = {0, 0, 0, 0};

    lwip_config_net(ip_addr, ip_addr, ip_addr);
    set_lwip_bit(LWIP_PRINT_FLAG);

    dhcp_start(&gnetif);

    lw_print("\r\n************************************************\r\n");
    lw_print(" DHCP example\r\n");
    lw_print("************************************************\r\n");

    dhcp_time = sys_now();

    while ((sys_now() - dhcp_time) < LWIP_DHCP_TIME)
    {
        /* Poll the driver, get any outstanding frames */
        ethernetif_input(&gnetif);

        /* Handle all system timeouts for all core protocols */
        sys_check_timeouts();

        /* Print DHCP progress */
        if(LwipPrintDHCP(&gnetif))
        {
            sscanf(ipaddr_ntoa(&gnetif.ip_addr), "%d.%d.%d.%d", &lwip_ipaddr[0], &lwip_ipaddr[1],
                &lwip_ipaddr[2], &lwip_ipaddr[3]);

            sscanf(ipaddr_ntoa(&gnetif.netmask), "%d.%d.%d.%d", &lwip_netmask[0], &lwip_netmask[1],
                &lwip_netmask[2], &lwip_netmask[3]);

            sscanf(ipaddr_ntoa(&gnetif.gw), "%d.%d.%d.%d", &lwip_gwaddr[0], &lwip_gwaddr[1],
                &lwip_gwaddr[2], &lwip_gwaddr[3]);

            break;
        }
    }

    clr_lwip_bit(LWIP_PRINT_FLAG);
}


SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
     getdynip, LwipDHCPTest, DHCP_Test);

#endif
