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
 * @file hc08.c
 * @brief Implement the connection Bluetooth adapter function, using HC08 device
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.07.12
 */

#include <adapter.h>
#include <at_agent.h>

#define HC08_DETECT_CMD			"AT"
#define HC08_DEFAULT_CMD		"AT+DEFAULT"
#define HC08_RESET_CMD			"AT+RESET"
#define HC08_CLEAR_CMD			"AT+CLEAR"
#define HC08_GET_DEVICE_INFO    "AT+RX"

#define HC08_GET_BAUDRATE_CMD   "AT+BAUD=?"
#define HC08_SET_BAUDRATE_CMD   "AT+BAUD=%u"
#define HC08_GET_CONNECTABLE    "AT+CONT=?"
#define HC08_SET_CONNECTABLE    "AT+CONT=%s"
#define HC08_GET_ROLE_CMD		"AT+ROLE=?"
#define HC08_SET_ROLE_CMD		"AT+ROLE=%s"
#define HC08_GET_ADDR_CMD		"AT+ADDR=?"
#define HC08_SET_ADDR_CMD		"AT+ADDR=%s"
#define HC08_GET_NAME_CMD       "AT+NAME=%s"
#define HC08_SET_NAME_CMD       "AT+NAME=?"

#define HC08_OK_RESP			"OK"

#define HC08_CMD_STR_DEFAULT_SIZE	64
#define HC08_RESP_DEFAULT_SIZE		64

enum Hc08AtCmd
{
    HC08_AT_CMD_DETECT = 0,
    HC08_AT_CMD_DEFAULT,
    HC08_AT_CMD_RESET,
    HC08_AT_CMD_CLEAR,
    HC08_AT_CMD_GET_DEVICE_INFO,
    HC08_AT_CMD_GET_BAUDRATE,
    HC08_AT_CMD_SET_BAUDRATE,
    HC08_AT_CMD_SET_CONNECTABLE,
    HC08_AT_CMD_GET_CONNECTABLE,
    HC08_AT_CMD_SET_ROLE,
    HC08_AT_CMD_GET_ROLE,
    HC08_AT_CMD_SET_ADDR,
    HC08_AT_CMD_GET_ADDR,
    HC08_AT_CMD_SET_NAME,
    HC08_AT_CMD_GET_NAME,
    HC08_AT_CMD_END,
};

static int Hc08CheckMacHex(char *hex, int len)
{
    int i = 0;
    while (i < len)
    {
        if ((hex[i] >= 'A' && hex[i] <= 'F') ||
            (hex[i] >= 'a' && hex[i] <= 'f') ||
            (hex[i] >= '0' && hex[i] <= '9'))
        {
            i++;
            continue;
        }
        return -1;
    }
    return 0;
}

//HC08 AT cmd function
static int Hc08AtConfigure(ATAgentType agent, enum Hc08AtCmd hc08_at_cmd, void *param, char *reply_info)
{
    const char *result_buf;
    char *connectable, *role;
    unsigned int baudrate;
    char reply_ok_flag = 1;
    char cmd_str[HC08_CMD_STR_DEFAULT_SIZE] = {0};

    ATReplyType reply = CreateATReply(64);
    if (NULL == reply) {
        printf("Hc08AtConfigure CreateATReply failed !\n");
        return -ENOMEMORY;
    }
    
    switch (hc08_at_cmd)
    {
    case HC08_AT_CMD_DETECT:
        AtSetReplyEndChar(agent, 0x4F, 0x4B);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_DETECT_CMD);
        break;
    case HC08_AT_CMD_DEFAULT:
        AtSetReplyEndChar(agent, 0x4F, 0x4B);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_DEFAULT_CMD);
        break;
    case HC08_AT_CMD_RESET:
        AtSetReplyEndChar(agent, 0x4F, 0x4B);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_RESET_CMD);
        break;
    case HC08_AT_CMD_CLEAR:
        AtSetReplyEndChar(agent, 0x4F, 0x4B);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_CLEAR_CMD);
        break;
    case HC08_AT_CMD_GET_DEVICE_INFO:
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_GET_DEVICE_INFO);
        reply_ok_flag = 0;
        break;
    case HC08_AT_CMD_SET_BAUDRATE:
        baudrate = *(unsigned int *)param;
        sprintf(cmd_str, HC08_SET_BAUDRATE_CMD, baudrate);
        strcat(cmd_str, ",N");
        AtSetReplyEndChar(agent, 0x4E, 0x45);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, cmd_str);
        reply_ok_flag = 0;
        break;
    case HC08_AT_CMD_GET_BAUDRATE:
        AtSetReplyEndChar(agent, 0x2C, 0x4E);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_GET_BAUDRATE_CMD);
        reply_ok_flag = 0;
        break;
    case HC08_AT_CMD_SET_CONNECTABLE:
        connectable = (char *)param;
        sprintf(cmd_str, HC08_SET_CONNECTABLE, connectable);
        AtSetReplyEndChar(agent, 0x4F, 0x4B);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, cmd_str);
        break;
    case HC08_AT_CMD_GET_CONNECTABLE:
        AtSetReplyEndChar(agent, 0x6C, 0x65);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_GET_CONNECTABLE);
        reply_ok_flag = 0;
        break;
    case HC08_AT_CMD_SET_ROLE:
        role = (char *)param;
        sprintf(cmd_str, HC08_SET_ROLE_CMD, role);
        AtSetReplyCharNum(agent, 1);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, cmd_str);
        break;
    case HC08_AT_CMD_GET_ROLE:
        AtSetReplyCharNum(agent, 1);
        ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_GET_ROLE_CMD);
        reply_ok_flag = 0;
        break;
    // case HC08_AT_CMD_SET_ADDR:
    //     ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_SET_ROLE_CMD);
    //     break;
    // case HC08_AT_CMD_GET_ADDR:
    //     ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_GET_ROLE_CMD);
    //     reply_ok_flag = 0;
    //     break;
    // case HC08_AT_CMD_SET_NAME:
    //     ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_SET_NAME_CMD);
    //     break;
    // case HC08_AT_CMD_GET_NAME:
    //     ATOrderSend(agent, REPLY_TIME_OUT, reply, HC08_GET_NAME_CMD);
    //     reply_ok_flag = 0;
    //     break;
    default:
        printf("hc08 do not support no.%d cmd\n", hc08_at_cmd);
        DeleteATReply(reply);
        return -1;
    }

    PrivTaskDelay(200);

    result_buf = GetReplyText(reply);

    if (reply_ok_flag) {
        if (!strstr(HC08_OK_RESP, result_buf)) {
            printf("%u cmd get reply OK failed:get reply %s\n", hc08_at_cmd, result_buf);
        }
    }

    if (reply_info) {
        strncpy(reply_info, result_buf, reply->reply_len);
    }

    DeleteATReply(reply);

    return 0;
}

static int Hc08Open(struct Adapter *adapter)
{   
    if (INSTALL == adapter->adapter_status) {
        printf("Hc08 has already been open\n");
        return 0;
    }
    
    //step1: open hc08 serial port
    adapter->fd = PrivOpen(ADAPTER_HC08_DRIVER, O_RDWR);
    if (adapter->fd < 0) {
        printf("Hc08Open get serial %s fd error\n", ADAPTER_HC08_DRIVER);
        return -1;
    }

    //step2: init AT agent
    if (!adapter->agent) {
        char *agent_name = "bluetooth_uart_client";
        printf("InitATAgent agent_name %s fd %u\n", agent_name, adapter->fd);
        if (EOK != InitATAgent(agent_name, adapter->fd, 512)) {
            printf("at agent init failed !\n");
            return -1;
        }
        ATAgentType at_agent = GetATAgent(agent_name);

        adapter->agent = at_agent;

        printf("Hc08Open adapter agent %p\n", adapter->agent);
    }

    PrivTaskDelay(200);

    ADAPTER_DEBUG("Hc08 open done\n");

    return 0;
}

static int Hc08Close(struct Adapter *adapter)
{
    return 0;
}

static int Hc08Ioctl(struct Adapter *adapter, int cmd, void *args)
{
    if (OPE_INT != cmd) {
        printf("Hc08Ioctl only support OPE_INT, do not support %d\n", cmd);
        return -1;
    }

    char hc08_baudrate[HC08_RESP_DEFAULT_SIZE] = {0};
    uint32_t baud_rate = *((uint32_t *)args);

    struct SerialDataCfg serial_cfg;
    memset(&serial_cfg, 0 ,sizeof(struct SerialDataCfg));
    serial_cfg.serial_baud_rate = baud_rate;
    serial_cfg.serial_data_bits = DATA_BITS_8;
    serial_cfg.serial_stop_bits = STOP_BITS_1;
    serial_cfg.serial_buffer_size = SERIAL_RB_BUFSZ;
    serial_cfg.serial_parity_mode = PARITY_NONE;
    serial_cfg.serial_bit_order = STOP_BITS_1;
    serial_cfg.serial_invert_mode = NRZ_NORMAL;
#ifdef ADAPTER_HC08_DRIVER_EXT_PORT
    serial_cfg.ext_uart_no = ADAPTER_HC08_DRIVER_EXT_PORT;
    serial_cfg.port_configure = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &serial_cfg;
    PrivIoctl(adapter->fd, OPE_INT, &ioctl_cfg);

    //Step1 : detect hc08 serial function
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_DETECT, NULL, NULL) < 0) {
        return -1;
    }

    //Step2 : set hc08 device serial baud, hc08_set_baud send "AT+BAUD=%s"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_SET_BAUDRATE, args, NULL) < 0) {
        return -1;
    }

    PrivTaskDelay(200);

    //Step3 : show hc08 device info, hc08_get send "AT+RX" response device info
    char device_info[HC08_RESP_DEFAULT_SIZE * 2] = {0};
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_GET_DEVICE_INFO, NULL, device_info) < 0) {
        return -1;
    }

    ADAPTER_DEBUG("Hc08 ioctl done\n");
    
    return 0;
}

static int Hc08SetAddr(struct Adapter *adapter, const char *ip, const char *gateway, const char *netmask)
{
    char mac_addr[HC08_RESP_DEFAULT_SIZE] = {0};
    
    //Step1 : hc08_get_addr send "AT+ADDR=?" response mac "XX,XX,XX,XX,XX,XX"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_GET_ADDR, NULL, mac_addr) < 0) {
        return -1;
    }

    printf("HC08 old mac addr: %s\n", mac_addr);

    //Step2 : hc08_set_addr send "AT+ADDR=%s" response "OK"
    if (!Hc08CheckMacHex((char *)gateway, 12)) {
        if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_SET_ADDR, (void *)gateway, NULL) < 0) {
            return -1;
        }
    }

    //Step3 : check mac addr, hc08_get_addr send "AT+ADDR=?" response mac "XX,XX,XX,XX,XX,XX"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_GET_ADDR, NULL, mac_addr) < 0) {
        return -1;
    }

    printf("HC08 new mac addr: %s\n", mac_addr);

    return 0;
}

static int Hc08Connect(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type)
{    
    char connect_status[HC08_RESP_DEFAULT_SIZE] = {0};
    
    //Step1 : hc08_detect send "AT" response "OK"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_DETECT, NULL, NULL) < 0) {
        return -1;
    }

    //Step2 : hc08_set_role send "AT+ROLE=%s" response "OK"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_SET_ROLE, ADAPTER_HC08_WORK_ROLE, NULL) < 0) {
        return -1;
    }

    //Step3 : hc08_connectable send "AT+CONT=0" response "OK"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_SET_CONNECTABLE, "0", NULL) < 0) {
        return -1;
    }

    //Step4 : check connectable status,hc08_connectable send "AT+CONT=?" response "Connectable"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_GET_CONNECTABLE, NULL, connect_status) < 0) {
        return -1;
    }   

    printf("Hc08Connect status %s\n", connect_status);

    return 0;
}

static int Hc08Send(struct Adapter *adapter, const void *buf, size_t len)
{
    x_err_t result = EOK;
    if (adapter->agent) {
        EntmSend(adapter->agent, (const char *)buf, len);
    } else {
        printf("Hc08Send can not find agent\n");
	}
    return 0;
}

static int Hc08Recv(struct Adapter *adapter, void *buf, size_t len)
{
    if (adapter->agent) {
        return EntmRecv(adapter->agent, (char *)buf, len, 40);
    } else {
        printf("Hc08Recv can not find agent\n");
	}
    
    return -1;
}

static int Hc08Disconnect(struct Adapter *adapter)
{
    char connect_status[HC08_RESP_DEFAULT_SIZE] = {0};

    //Step1 : hc08_detect send "AT" response "OK"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_DETECT, NULL, NULL) < 0) {
        return -1;
    }

    //Step2 : hc08_connectable send "AT+CONT=1" response "OK"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_SET_CONNECTABLE, "1", NULL) < 0) {
        return -1;
    }    

    //Step3 : check connectable status,hc08_connectable send "AT+CONT=?" response "Not-Connectable"
    if (Hc08AtConfigure(adapter->agent, HC08_AT_CMD_GET_CONNECTABLE, NULL, connect_status) < 0) {
        return -1;
    }   

    printf("Hc08Disconnect status %s\n", connect_status);
    
    return 0;
}

static const struct IpProtocolDone hc08_done = 
{
    .open = Hc08Open,
    .close = Hc08Close,
    .ioctl = Hc08Ioctl,
    .setup = NULL,
    .setdown = NULL,
    .setaddr = Hc08SetAddr,
    .setdns = NULL,
    .setdhcp = NULL,
    .ping = NULL,
    .netstat = NULL,
    .connect = Hc08Connect,
    .send = Hc08Send,
    .recv = Hc08Recv,
    .disconnect = Hc08Disconnect,
};

AdapterProductInfoType Hc08Attach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = PrivMalloc(sizeof(struct AdapterProductInfo));
    if (!product_info) {
        printf("Hc08Attach malloc product_info error\n");
        free(product_info);
        return NULL;
    }
    
    if ("M" == ADAPTER_HC08_WORK_ROLE) {
        adapter->net_role = MASTER;
    } else if ("S" == ADAPTER_HC08_WORK_ROLE) {
        adapter->net_role = SLAVE;
    } else {
        adapter->net_role = ROLE_NONE;
    }

    strcpy(product_info->model_name, ADAPTER_BLUETOOTH_HC08);

    product_info->model_done = (void *)&hc08_done;

    return product_info;
}