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
 * @file ec200t.c
 * @brief Implement the connection 4G adapter function, using EC200T device
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.07.08
 */

#include <adapter.h>
#include <at_agent.h>

#define EC200T_AT_MODE_CMD          "+++"
#define EC200T_GET_CCID_CMD         "AT+CCID\r\n"
#define EC200T_GET_CPIN_CMD         "AT+CPIN?\r\n"
#define EC200T_GET_CREG_CMD         "AT+CREG?\r\n"
#define EC200T_OPEN_SOCKET_CMD      "AT+QIOPEN=1,%u"
#define EC200T_CLOSE_SOCKET_CMD     "AT+QICLOSE=%u\r\n"
#define EC200T_ACTIVE_PDP_CMD       "AT+QIACT=1\r\n"
#define EC200T_DEACTIVE_PDP_CMD     "AT+QIDEACT=1\r\n"
#define EC200T_CFG_TCP_CMD          "AT+QICSGP"

#define EC200T_OK_REPLY             "OK"
#define EC200T_READY_REPLY          "READY"
#define EC200T_CREG_REPLY           ",1"
#define EC200T_CONNECT_REPLY        "CONNECT"

#define TRY_TIMES 10

#ifdef ADD_NUTTX_FETURES
static void Ec200tPowerSet(void){ return; }
#else
static void Ec200tPowerSet(void)
{
    int pin_fd;
    pin_fd = PrivOpen(ADAPTER_EC200T_PIN_DRIVER, O_RDWR);
    if (pin_fd < 0) {
        printf("open %s error\n", ADAPTER_EC200T_PIN_DRIVER);
        return;
    }

    struct PinParam pin_param;
    pin_param.cmd = GPIO_CONFIG_MODE;
    pin_param.mode = GPIO_CFG_OUTPUT;
    pin_param.pin = ADAPTER_EC200T_PWRKEY;

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = PIN_TYPE;
    ioctl_cfg.args = &pin_param;
    PrivIoctl(pin_fd, OPE_CFG, &ioctl_cfg);

    struct PinStat pin_stat;
    pin_stat.pin = ADAPTER_EC200T_PWRKEY;
    pin_stat.val = GPIO_HIGH;
    PrivWrite(pin_fd, &pin_stat, 1);

    PrivTaskDelay(600);//at least 500ms

    pin_stat.val = GPIO_LOW;
    PrivWrite(pin_fd, &pin_stat, 1);

    PrivClose(pin_fd);

    PrivTaskDelay(10000);
}
#endif

static int Ec200tOpen(struct Adapter *adapter)
{
    /*step1: open ec200t serial port*/
    adapter->fd = PrivOpen(ADAPTER_EC200T_DRIVER, O_RDWR);
    if (adapter->fd < 0) {
        printf("Ec200tOpen get serial %s fd error\n", ADAPTER_EC200T_DRIVER);
        return -1;
    }

    /*step2: init AT agent*/
    if (!adapter->agent) {
        char *agent_name = "4G_uart_client";
        if (0 != InitATAgent(agent_name, adapter->fd, 512)) {
            printf("at agent init failed !\n");
            return -1;
        }
        ATAgentType at_agent = GetATAgent(agent_name);

        adapter->agent = at_agent;
    }

    PrivTaskDelay(2500);
    
    ADAPTER_DEBUG("Ec200t open done\n");

    return 0;
}

static int Ec200tClose(struct Adapter *adapter)
{
    int ret = 0;
    uint8_t ec200t_cmd[64];
    
    if (!adapter->agent) {
        printf("Ec200tClose AT agent NULL\n");
        return -1;
    }

    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B);

    /*step1: serial write "+++", quit transparent mode*/
    ATOrderSend(adapter->agent, REPLY_TIME_OUT, NULL, "+++");

    /*step2: serial write "AT+QICLOSE", close socket connect before open socket*/
    memset(ec200t_cmd, 0, sizeof(ec200t_cmd));
    sprintf(ec200t_cmd, EC200T_CLOSE_SOCKET_CMD, adapter->socket.socket_id);

    ret = AtCmdConfigAndCheck(adapter->agent, ec200t_cmd, EC200T_OK_REPLY);
    if (ret < 0) {
        goto out;
    }

    /*step3: serial write "AT+QIDEACT", close TCP net before open socket*/
    ret = AtCmdConfigAndCheck(adapter->agent, EC200T_DEACTIVE_PDP_CMD, EC200T_OK_REPLY);
    if (ret < 0) {
        goto out;
    }

out:
    /*step4: close ec200t serial port*/
    PrivClose(adapter->fd);

    /*step5: power down ec200t*/
    Ec200tPowerSet();
    
    return ret;
}


#ifdef ADD_NUTTX_FETURES
static int Ec200tIoctl(struct Adapter *adapter, int cmd, void *args){ return 0;}
#else
static int Ec200tIoctl(struct Adapter *adapter, int cmd, void *args)
{
    if (OPE_INT != cmd) {
        printf("Ec200tIoctl only support OPE_INT, do not support %d\n", cmd);
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
    serial_cfg.serial_bit_order = STOP_BITS_1;
    serial_cfg.serial_invert_mode = NRZ_NORMAL;
#ifdef ADAPTER_EC200T_DRIVER_EXT_PORT
    serial_cfg.ext_uart_no = ADAPTER_EC200T_DRIVER_EXT_PORT;
    serial_cfg.port_configure = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &serial_cfg;
    PrivIoctl(adapter->fd, OPE_INT, &ioctl_cfg);

    Ec200tPowerSet();
    
    return 0;
}
#endif

static int Ec200tConnect(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type)
{
    int ret = 0;
    int try = 0;
    uint8_t ec200t_cmd[64];

    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B);

    /*step1: serial write "+++", quit transparent mode*/
    ATOrderSend(adapter->agent, REPLY_TIME_OUT, NULL, "+++");

    /*step2: serial write "AT+CCID", get SIM ID*/
    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, EC200T_GET_CCID_CMD, EC200T_OK_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    /*step3: serial write "AT+CPIN?", check SIM status*/
    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, EC200T_GET_CPIN_CMD, EC200T_READY_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    /*step4: serial write "AT+CREG?", check whether registered to GSM net*/
    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, EC200T_GET_CREG_CMD, EC200T_CREG_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    /*step5: serial write "AT+QICSGP", connect to China Mobile using ipv4 or ipv6*/
    memset(ec200t_cmd, 0, sizeof(ec200t_cmd));

    if (IPV4 == ip_type) {
        strcpy(ec200t_cmd, "AT+QICSGP=1,1,\"CMNET\",\"\",\"\",1\r\n");
    } else if (IPV6 == ip_type) {
        strcpy(ec200t_cmd, "AT+QICSGP=1,2,\"CMNET\",\"\",\"\",1\r\n");
    }
    
    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, ec200t_cmd, EC200T_OK_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    /*step6: serial write "AT+QICLOSE", close socket connect before open socket*/
    memset(ec200t_cmd, 0, sizeof(ec200t_cmd));
    sprintf(ec200t_cmd, EC200T_CLOSE_SOCKET_CMD, adapter->socket.socket_id);
    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, ec200t_cmd, EC200T_OK_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    /*step7: serial write "AT+QIDEACT", close TCP net before open socket*/
    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, EC200T_DEACTIVE_PDP_CMD, EC200T_OK_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    /*step8: serial write "AT+QIACT", open TCP net*/
    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, EC200T_ACTIVE_PDP_CMD, EC200T_OK_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    /*step9: serial write "AT+QIOPEN", connect socket using TCP*/
    memset(ec200t_cmd, 0, sizeof(ec200t_cmd));
    sprintf(ec200t_cmd, EC200T_OPEN_SOCKET_CMD, adapter->socket.socket_id);
    strcat(ec200t_cmd, ",\"TCP\",\"");
    strcat(ec200t_cmd, ip);
    strcat(ec200t_cmd, "\",");
    strcat(ec200t_cmd, port);
    strcat(ec200t_cmd, ",0,2\r\n");

    AtSetReplyEndChar(adapter->agent, 0x43, 0x54);

    for(try = 0; try < TRY_TIMES; try++){
        ret = AtCmdConfigAndCheck(adapter->agent, ec200t_cmd, EC200T_CONNECT_REPLY);
        if (ret == 0) {
            break;
        }
    }
    if (ret < 0) {
        goto out;
    }

    ADAPTER_DEBUG("Ec200t connect TCP done\n");

    return 0;

out:
    ADAPTER_DEBUG("Ec200t connect TCP failed. Power down\n");
    Ec200tPowerSet();
    return -1;
}

static int Ec200tSend(struct Adapter *adapter, const void *buf, size_t len)
{
    if (adapter->agent) {
        EntmSend(adapter->agent, (const char *)buf, len);
    } else {
        printf("Ec200tSend can not find agent\n");
    }
    return 0;
}

static int Ec200tRecv(struct Adapter *adapter, void *buf, size_t len)
{
    if (adapter->agent) {
        return EntmRecv(adapter->agent, (char *)buf, len, 6);
    } else {
        printf("Ec200tRecv can not find agent\n");
	}
    
    return -1;
}

static int Ec200tDisconnect(struct Adapter *adapter)
{
    int ret = 0;
    uint8_t ec200t_cmd[64];

    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B);
    
    /*step1: serial write "+++", quit transparent mode*/
    ATOrderSend(adapter->agent, REPLY_TIME_OUT, NULL, "+++");

    /*step2: serial write "AT+QICLOSE", close socket connect before open socket*/
    memset(ec200t_cmd, 0, sizeof(ec200t_cmd));
    sprintf(ec200t_cmd, EC200T_CLOSE_SOCKET_CMD, adapter->socket.socket_id);
    ret = AtCmdConfigAndCheck(adapter->agent, ec200t_cmd, EC200T_OK_REPLY);
    if (ret < 0) {
        goto out;
    }

    ADAPTER_DEBUG("Ec200t disconnect TCP done\n");

    return 0;

out:
    ADAPTER_DEBUG("Ec200t disconnect TCP failed. Power down\n");
    Ec200tPowerSet();
    return -1;
}

static const struct IpProtocolDone ec200t_done = 
{
    .open = Ec200tOpen,
    .close = Ec200tClose,
    .ioctl = Ec200tIoctl,
    .setup = NULL,
    .setdown = NULL,
    .setaddr = NULL,
    .setdns = NULL,
    .setdhcp = NULL,
    .ping = NULL,
    .netstat = NULL,
    .connect = Ec200tConnect,
    .send = Ec200tSend,
    .recv = Ec200tRecv,
    .disconnect = Ec200tDisconnect,
};

AdapterProductInfoType Ec200tAttach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = PrivMalloc(sizeof(struct AdapterProductInfo));
    if (!product_info) {
        printf("Ec200tAttach malloc product_info error\n");
        PrivFree(product_info);
        return NULL;
    }

    strcpy(product_info->model_name, ADAPTER_4G_EC200T);

    product_info->model_done = (void *)&ec200t_done;

    return product_info;
}
