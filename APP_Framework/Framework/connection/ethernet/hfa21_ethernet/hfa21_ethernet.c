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
 * @file hfa21_ethernet.c
 * @brief Implement the connection ethernet adapter function, using HFA21 device
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.10.15
 */

#include <adapter.h>
#include <at_agent.h>

#define HFA21_ETHERNET_AT_CMD                  "+++"
#define HFA21_ETHERNET_AT_CONFIRM_CMD          "a"
#define HFA21_ETHERNET_AT_FCLR_CMD             "AT+FCLR\r"
#define HFA21_ETHERNET_AT_FEPHY_CMD            "AT+FEPHY=on\r"
#define HFA21_ETHERNET_AT_ENABLE_WANN_CMD      "AT+FVEW=enable\r"
#define HFA21_ETHERNET_AT_DISABLE_WANN_CMD     "AT+FVEW=disable\r"
#define HFA21_ETHERNET_AT_RELD_CMD             "AT+RELD\r"
#define HFA21_ETHERNET_AT_WMODE_CMD            "AT+WMODE\r"
#define HFA21_ETHERNET_AT_WANN_CMD             "AT+WANN\r"
#define HFA21_ETHERNET_AT_SET_WANN_CMD         "AT+WANN=%s,%s,%s,%s\r"
#define HFA21_ETHERNET_AT_PING_CMD             "AT+PING=%s\r"
#define HFA21_ETHERNET_AT_NETP_CMD             "AT+NETP=%s,%s,%s,%s\r"
#define HFA21_ETHERNET_AT_REBOOT_CMD           "AT+Z\r"

#define HFA21_ETHERNET_OK_REPLY        "+ok"

static int Hfa21EthernetSetDown(struct Adapter *adapter_at);

/**
 * @description: enter AT command mode
 * @param at_agent - ethernet device agent pointer
 * @return success: 0
 */
static int Hfa21EthernetInitAtCmd(ATAgentType at_agent)
{
    ATOrderSend(at_agent, REPLY_TIME_OUT, NULL, HFA21_ETHERNET_AT_CMD);
    PrivTaskDelay(100);

    ATOrderSend(at_agent, REPLY_TIME_OUT, NULL, HFA21_ETHERNET_AT_CONFIRM_CMD);
    PrivTaskDelay(500);

    return 0;
}

/**
 * @description: Open HFA21 uart function
 * @param adapter - ethernet device pointer
 * @return success: 0, failure: -1
 */
static int Hfa21EthernetOpen(struct Adapter *adapter)
{
    /*step1: open hfa21 serial port*/
    adapter->fd = PrivOpen(ADAPTER_HFA21_DRIVER, O_RDWR);
    if (adapter->fd < 0) {
        printf("Hfa21EthernetOpen get serial %s fd error\n", ADAPTER_HFA21_DRIVER);
        return -1;
    }

    /*step2: init AT agent*/
    if (!adapter->agent) {
        char *agent_name = "ethernet_uart_client";
        if (EOK != InitATAgent(agent_name, adapter->fd, 512)) {
            printf("at agent init failed !\n");
            return -1;
        }
        ATAgentType at_agent = GetATAgent(agent_name);

        adapter->agent = at_agent;
    }

    ADAPTER_DEBUG("Hfa21Ethernet open done\n");

    return 0;
}

/**
 * @description: Close HFA21 ethernet and uart function
 * @param adapter - ethernet device pointer
 * @return success: 0
 */
static int Hfa21EthernetClose(struct Adapter *adapter)
{
    return Hfa21EthernetSetDown(adapter);
}

/**
 * @description: send data to adapter
 * @param adapter - ethernet device pointer
 * @param data - data buffer
 * @param data - data length
 * @return success: 0
 */
static int Hfa21EthernetSend(struct Adapter *adapter, const void *data, size_t len)
{
    int result = 0;
    if (adapter->agent) {
        EntmSend(adapter->agent, (const char *)data, len);
    }else {
        printf("Hfa21EthernetSend can not find agent!\n");
	}

__exit:

    return result;
}

/**
 * @description: receive data from adapter
 * @param adapter - ethernet device pointer
 * @param data - data buffer
 * @param data - data length
 * @return success: 0
 */
static int Hfa21EthernetReceive(struct Adapter *adapter, void *rev_buffer, size_t buffer_len)
{
    int result = 0;

    if (adapter->agent) {
        return EntmRecv(adapter->agent, (char *)rev_buffer, buffer_len, 40000);
    } else {
        printf("Hfa21EthernetReceive can not find agent!\n");
	}

__exit:

    return result;
}

/**
 * @description: connnect Ethernet to internet
 * @param adapter - Ethernet device pointer
 * @return success: 0
 */
static int Hfa21EthernetSetUp(struct Adapter *adapter)
{
    int ret = 0;

    AtSetReplyLrEnd(adapter->agent, 1);

    /* wait hfa21 device startup finish */
    PrivTaskDelay(5000);

    /*Step1 : enter AT mode*/
    Hfa21EthernetInitAtCmd(adapter->agent);

    PrivTaskDelay(1000);

    /*Step2 : FCLR reboot*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_FCLR_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    PrivTaskDelay(10000);

    Hfa21EthernetInitAtCmd(adapter->agent);

    /*Step3 : FEPHY enable phy function*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_FEPHY_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    /*Step4 : FVEW disable WANN function, ethernet work at LANN mode*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_DISABLE_WANN_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    /*Step5 : RELD enable F-AT cmd*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_RELD_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    PrivTaskDelay(10000);

    Hfa21EthernetInitAtCmd(adapter->agent);

    /*Step6 : AT+WMODE check work mode, AP or STA*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_WMODE_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    /*Step7 : AT+WANN check if get ip、netmask、gateway*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_WANN_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    /*Step8 : AT+Z reboot hfa21 device*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_REBOOT_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    PrivTaskDelay(10000);

    return ret;

__exit:
    Hfa21EthernetSetDown(adapter);

    return -1;
}

/**
 * @description: disconnnect ethernet from internet
 * @param adapter - ethernet device pointer
 * @return success: 0
 */
static int Hfa21EthernetSetDown(struct Adapter *adapter)
{
    Hfa21EthernetInitAtCmd(adapter->agent);

    ATOrderSend(adapter->agent, REPLY_TIME_OUT, NULL, HFA21_ETHERNET_AT_FCLR_CMD);
    PrivTaskDelay(20000);

    return 0;
}

/**
 * @description: set ethernet ip/gateway/netmask address(in sta mode) working at WANN mode
 * @param adapter - ethernet device pointer
 * @param ip - ip address
 * @param gateway - gateway address
 * @param netmask - netmask address
 * @return success: 0, failure: -ENOMEMORY or -1
 */
static int Hfa21EthernetSetAddr(struct Adapter *adapter, const char *ip, const char *gateway, const char *netmask)
{
    int ret = 0;
    uint8_t hfa21_ethernet_cmd[64];

    /*Step1 : enter AT mode*/
    Hfa21EthernetInitAtCmd(adapter->agent);

    /*Step2 : set mode、ip、netmask and gateway*/
    memset(hfa21_ethernet_cmd, 0, sizeof(hfa21_ethernet_cmd));
    sprintf(hfa21_ethernet_cmd, HFA21_ETHERNET_AT_SET_WANN_CMD, "DHCP", ip, netmask, gateway);
    ret = AtCmdConfigAndCheck(adapter->agent, hfa21_ethernet_cmd, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    /*Step3 : AT+WANN check if set ip、netmask、gateway successfully*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_WANN_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    /*Step4 : AT+Z reboot hfa21 device*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_REBOOT_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    PrivTaskDelay(10000);

    return ret;

__exit:
    Hfa21EthernetSetDown(adapter);

    return -1;
}

/**
 * @description: ethernet ping function
 * @param adapter - ethernet device pointer
 * @param destination - domain name or ip address
 * @return success: 0, failure: -1
 */
static int Hfa21EthernetPing(struct Adapter *adapter, const char *destination)
{
    int ret = 0;
    
    char *ping_result = (char *) PrivCalloc(1, 17);
    char *dst = (char *) PrivCalloc(1, 17);

    strcpy(dst, destination);
    strcat(dst, "\r");

    printf("Hfa21EthernetPing [%s]\n", dst);

    /*Step1 : enter AT mode*/
    Hfa21EthernetInitAtCmd(adapter->agent);

    /*Step2 : ping dst ip address*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_PING_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    /*Step3 : AT+Z reboot hfa21 device*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_REBOOT_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    return ret;

__exit:
    Hfa21EthernetSetDown(adapter);

    return -1;
}

/**
 * @description: ethernet connect function
 * @param adapter - ethernet device pointer
 * @param net_role - net role, CLIENT or SERVER
 * @param ip - ip address
 * @param port - port num
 * @param ip_type - ip type, IPV4 or IPV6
 * @return success: 0, failure: -1
 */
static int Hfa21EthernetConnect(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type)
{
    int ret = 0;
    char hfa21_ethernet_cmd[128];
    char net_role_string[7] = {0};

    /*Step1 : enter AT mode*/
    Hfa21EthernetInitAtCmd(adapter->agent);

    if (CLIENT == net_role) {
        strcpy(net_role_string, "CLIENT");
    } else if (SERVER == net_role) {
        strcpy(net_role_string, "SERVER");
    } else {
        printf("Hfa21EthernetConnect do not support %d net type\n", net_role);
        return -1;
    }

    if (IPV4 == ip_type) {
        /*to do*/
    } else if (IPV6 == ip_type) {
        /*to do*/
    } else {
        printf("Hfa21EthernetConnect do not support %d ip type\n", ip_type);
        return -1;
    }

    /*Step2 : create tcp connect*/
    memset(hfa21_ethernet_cmd, 0, sizeof(hfa21_ethernet_cmd));
    sprintf(hfa21_ethernet_cmd, HFA21_ETHERNET_AT_NETP_CMD, "TCP", net_role_string, port, ip);
    ret = AtCmdConfigAndCheck(adapter->agent, hfa21_ethernet_cmd, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    adapter->net_role = net_role;

    /*Step3 : AT+Z reboot hfa21 device*/
    ret = AtCmdConfigAndCheck(adapter->agent, HFA21_ETHERNET_AT_REBOOT_CMD, HFA21_ETHERNET_OK_REPLY);
    if (ret < 0) {
        goto __exit;
    }

    PrivTaskDelay(10000);

    return ret;

__exit:
    Hfa21EthernetSetDown(adapter);

    return -1;
}

static int Hfa21EthernetIoctl(struct Adapter *adapter, int cmd, void *args)
{
    if (OPE_INT != cmd) {
        printf("Hfa21EthernetIoctl only support OPE_INT, do not support %d\n", cmd);
        return -1;
    }

    uint32_t baud_rate = *((uint32_t *)args);

    struct SerialDataCfg serial_cfg;
    memset(&serial_cfg, 0 ,sizeof(struct SerialDataCfg));
    serial_cfg.serial_baud_rate = baud_rate;
    serial_cfg.serial_data_bits = DATA_BITS_8;
    serial_cfg.serial_stop_bits = STOP_BITS_1;
    serial_cfg.serial_buffer_size = SERIAL_RB_BUFSZ;
    serial_cfg.serial_parity_mode = PARITY_NONE;
    serial_cfg.serial_bit_order = BIT_ORDER_LSB;
    serial_cfg.serial_invert_mode = NRZ_NORMAL;
#ifdef ADAPTER_HFA21_DRIVER_EXT_PORT
    serial_cfg.ext_uart_no = ADAPTER_HFA21_DRIVER_EXT_PORT;
    serial_cfg.port_configure = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &serial_cfg;
    PrivIoctl(adapter->fd, OPE_INT, &ioctl_cfg);

    printf("Hfa21EthernetIoctl success\n");
    return 0;
}

static const struct IpProtocolDone hfa21_ethernet_done =
{
    .open = Hfa21EthernetOpen,
    .close =  Hfa21EthernetClose,
    .ioctl = Hfa21EthernetIoctl,
    .setup = Hfa21EthernetSetUp,
    .setdown = Hfa21EthernetSetDown,
    .setaddr = Hfa21EthernetSetAddr,
    .setdns = NULL,
    .setdhcp = NULL,
    .ping = Hfa21EthernetPing,
    .netstat = NULL,
    .connect = Hfa21EthernetConnect,
    .send = Hfa21EthernetSend,
    .recv = Hfa21EthernetReceive,
    .disconnect = NULL,
};

/**
 * @description: Register ethernet device hfa21
 * @return success: product_info, failure: NULL
 */
AdapterProductInfoType Hfa21EthernetAttach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = PrivMalloc(sizeof(struct AdapterProductInfo));
    if (!product_info) {
        printf("Hfa21EthernetAttach Attach malloc product_info error\n");
        PrivFree(product_info);
        return NULL;
    }

    strcpy(product_info->model_name, ADAPTER_ETHERNET_HFA21);

    product_info->model_done = (void *)&hfa21_ethernet_done;

    return product_info;
}
