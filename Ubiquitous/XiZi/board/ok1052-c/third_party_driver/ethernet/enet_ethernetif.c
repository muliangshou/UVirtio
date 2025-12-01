/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
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
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file enet_ethernetif.c
 * @brief ethernet drivers
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.11.11
 */

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/ethip6.h"
#include "netif/etharp.h"
#include "netif/ppp/pppoe.h"
#include "lwip/igmp.h"
#include "lwip/mld6.h"

#if USE_RTOS && defined(FSL_RTOS_FREE_RTOS)
//#include "FreeRTOS.h"
//#include "event_groups.h"
#endif

#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "enet_ethernetif_priv.h"

#include "fsl_enet.h"
#include "fsl_phy.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"

#include "sys_arch.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void enet_delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 1000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void Time_Update_LwIP(void)
{
}

void ethernetif_clk_init(void)
{
    const clock_enet_pll_config_t config = {.enableClkOutput = true, .enableClkOutput25M = false, .loopDivider = 1};
    CLOCK_InitEnetPll(&config);
    SysTick_Config(SystemCoreClock / TICK_PER_SECOND);
}

void ethernetif_gpio_init(void)
{
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

    GPIO_PinInit(GPIO1, 3, &gpio_config);
    /* pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput(GPIO1, 3, 0);
    enet_delay();
    GPIO_WritePinOutput(GPIO1, 3, 1);
}

void ETH_BSP_Config(void)
{
    static int flag = 0;
    if(flag == 0)
    {
        ethernetif_clk_init();
        ethernetif_gpio_init();
        flag = 1;
    }
}

void ethernetif_phy_init(struct ethernetif *ethernetif,
                         const ethernetif_config_t *ethernetifConfig,
                         enet_config_t *config)
{
    uint32_t sysClock;
    status_t status;
    bool link = false;
    uint32_t count = 0;
    phy_speed_t speed;
    phy_duplex_t duplex;

    sysClock = CLOCK_GetFreq(ethernetifConfig->clockName);

    LWIP_PLATFORM_DIAG(("Initializing PHY...\r\n"));

    while ((count < ENET_ATONEGOTIATION_TIMEOUT) && (!link))
    {
        status = PHY_Init(*ethernetif_enet_ptr(ethernetif), ethernetifConfig->phyAddress, sysClock);

        if (kStatus_Success == status)
        {
            PHY_GetLinkStatus(*ethernetif_enet_ptr(ethernetif), ethernetifConfig->phyAddress, &link);
        }
        else if (kStatus_PHY_AutoNegotiateFail == status)
        {
            LWIP_PLATFORM_DIAG(("PHY Auto-negotiation failed. Please check the ENET cable connection and link partner setting."));
        }
        else
        {
            LWIP_ASSERT("\r\nCannot initialize PHY.\r\n", 0);
        }

        count++;
    }

    if (link)
    {
        /* Get the actual PHY link speed. */
        PHY_GetLinkSpeedDuplex(*ethernetif_enet_ptr(ethernetif), ethernetifConfig->phyAddress, &speed, &duplex);
        /* Change the MII speed and duplex for actual link status. */
        config->miiSpeed = (enet_mii_speed_t)speed;
        config->miiDuplex = (enet_mii_duplex_t)duplex;
    }
#if 0 /* Disable assert. If initial auto-negation is timeout, \ \
         the ENET is set to default (100Mbs and full-duplex). */
    else
    {
        LWIP_ASSERT("\r\nGiving up PHY initialization. Please check the ENET cable connection and link partner setting and reset the board.\r\n", 0);
    }
#endif
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function ethernetif_linkinput() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */

void ethernetif_input(struct netif *netif)
{
    struct pbuf *p;
    err_t ret = 0;

    LWIP_ASSERT("netif != NULL", (netif != NULL));

    /* move received packet into a new pbuf */
    while ((p = ethernetif_linkinput(netif)) != NULL)
    {
        /* pass all packets to ethernet_input, which decides what packets it supports */
        if ((ret = netif->input(p, netif)) != ERR_OK)
        {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
            lw_print("lw: [%s] ret %d p %p\n", __func__, ret, p);
            pbuf_free(p);
            p = NULL;
        }
    }
}

static ENET_Type *ethernetif_get_enet_base(const uint8_t enetIdx)
{
    ENET_Type* enets[] = ENET_BASE_PTRS;
    int arrayIdx;
    int enetCount;

    for (arrayIdx = 0, enetCount = 0; arrayIdx < ARRAY_SIZE(enets); arrayIdx++)
    {
        if (enets[arrayIdx] != 0U)    /* process only defined positions */
        {                             /* (some SOC headers count ENETs from 1 instead of 0) */
            if (enetCount == enetIdx)
            {
                return enets[arrayIdx];
            }
            enetCount++;
        }
    }
    return NULL;
}

err_t ethernetif_init(struct netif *netif, struct ethernetif *ethernetif,
                      const uint8_t enetIdx,
                      const ethernetif_config_t *ethernetifConfig)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));
    LWIP_ASSERT("ethernetifConfig != NULL", (ethernetifConfig != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
     * Initialize the snmp variables and counters inside the struct netif.
     * The last argument should be replaced with your link speed, in units
     * of bits per second.
     */
    MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->state = ethernetif;
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
/* We directly use etharp_output() here to save a function call.
 * You can instead declare your own function an call etharp_output()
 * from it if you have to do some checks before sending (e.g. if link
 * is available...) */
#if LWIP_IPV4
    netif->output = etharp_output;
#endif
#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
    netif->linkoutput = ethernetif_linkoutput;

#if LWIP_IPV4 && LWIP_IGMP
    netif_set_igmp_mac_filter(netif, ethernetif_igmp_mac_filter);
    netif->flags |= NETIF_FLAG_IGMP;
#endif
#if LWIP_IPV6 && LWIP_IPV6_MLD
    netif_set_mld_mac_filter(netif, ethernetif_mld_mac_filter);
    netif->flags |= NETIF_FLAG_MLD6;
#endif

    /* Init ethernetif parameters.*/
    *ethernetif_enet_ptr(ethernetif) = ethernetif_get_enet_base(enetIdx);
    LWIP_ASSERT("*ethernetif_enet_ptr(ethernetif) != NULL", (*ethernetif_enet_ptr(ethernetif) != NULL));

    /* set MAC hardware address length */
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* set MAC hardware address */
    memcpy(netif->hwaddr, ethernetifConfig->macAddress, NETIF_MAX_HWADDR_LEN);

    /* maximum transfer unit */
    netif->mtu = 1500; /* TODO: define a config */

    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    /* ENET driver initialization.*/
    ethernetif_enet_init(netif, ethernetif, ethernetifConfig);

#if LWIP_IPV6 && LWIP_IPV6_MLD
    /*
     * For hardware/netifs that implement MAC filtering.
     * All-nodes link-local is handled by default, so we must let the hardware know
     * to allow multicast packets in.
     * Should set mld_mac_filter previously. */
    if (netif->mld_mac_filter != NULL)
    {
        ip6_addr_t ip6_allnodes_ll;
        ip6_addr_set_allnodes_linklocal(&ip6_allnodes_ll);
        netif->mld_mac_filter(netif, &ip6_allnodes_ll, NETIF_ADD_MAC_FILTER);
    }
#endif /* LWIP_IPV6 && LWIP_IPV6_MLD */

    return ERR_OK;
}

