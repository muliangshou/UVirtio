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
 * @file e18.c
 * @brief Implement the connection zigbee adapter function, using E18 device
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.09.15
 */

#include <adapter.h>
#include <at_agent.h>
#include "../adapter_zigbee.h"
#include "e18.h"

char *cmd_hex2at = "+++";
char *cmd_exit = "AT+EXIT";
char *cmd_ask_panid = "AT+PANID=?";
char *cmd_set_panid = "AT+PANID=A1B2";
char *cmd_ask_mode = "AT+MODE=?";
char *cmd_mode_as_tt = "AT+MODE=1";       /*mode 1:transparent transmission*/
char *cmd_mode_as_stt = "AT+MODE=2";      /*mode 2:Semi transparent transmission*/
char *cmd_mode_as_protocol = "AT+MODE=3"; /*mode 3:Protocol mode*/
char *cmd_ask_role = "AT+DEV=?";
char *cmd_role_as_c = "AT+DEV=C"; /*set device type for coordinater*/
char *cmd_role_as_e = "AT+DEV=E"; /*set device type for end device*/
char *cmd_role_as_r = "AT+DEV=R"; /*set device type for router*/
char *cmd_set_ch = "AT+CH=11";    /*set channel as 11*/

#define E18_AS_HEX_MODE 0
#define E18_AS_AT_MODE  1

static int E18HardwareModeGet()
{
    int ret = 0;
    int pin_fd;

    pin_fd = PrivOpen(ADAPTER_BC28_PIN_DRIVER, O_RDWR);

    struct PinStat pin_stat;
    pin_stat.pin = ADAPTER_E18_MODEPIN;

    ret = PrivRead(pin_fd, &pin_stat, 1);

    PrivTaskDelay(200);

    PrivClose(pin_fd);

    if(pin_stat.val == GPIO_HIGH) {
        printf(" E18 as AT mode\n");
        return E18_AS_AT_MODE;
    } else {
        printf(" E18 as HEX mode\n");
        return E18_AS_HEX_MODE;
    } 
}

#ifdef ADD_NUTTX_FETURES
static int E18UartOpen(struct Adapter *adapter)
{
    adapter->fd = PrivOpen(ADAPTER_E18_DRIVER, O_RDWR);
    if (adapter->fd < 0) {
        printf("E18UartSetUp get serial %s fd error\n", ADAPTER_E18_DRIVER);
        return -1;
    }

    return adapter->fd;
}
#else
static int E18UartOpen(struct Adapter *adapter)
{
    if (NULL == adapter) {
        return -1;
    }

    /* Open device in read-write mode */
    adapter->fd = PrivOpen(ADAPTER_E18_DRIVER,O_RDWR);
    if (adapter->fd < 0) {
        printf("E18UartSetUp get serial %s fd error\n", ADAPTER_E18_DRIVER);
        return -1;
    }
    /* set serial config, serial_baud_rate = 115200 */

    struct SerialDataCfg cfg;
    memset(&cfg, 0 ,sizeof(struct SerialDataCfg));

    cfg.serial_baud_rate = BAUD_RATE_115200;
    cfg.serial_data_bits = DATA_BITS_8;
    cfg.serial_stop_bits = STOP_BITS_1;
    cfg.serial_parity_mode = PARITY_NONE;
    cfg.serial_bit_order = BIT_ORDER_LSB;
    cfg.serial_invert_mode = NRZ_NORMAL;
    cfg.serial_buffer_size = SERIAL_RB_BUFSZ;

    /*aiit board use ch438, so it needs more serial configuration*/
#ifdef ADAPTER_E18_DRIVER_EXTUART
    cfg.ext_uart_no         = ADAPTER_E18_DRIVER_EXT_PORT;
    cfg.port_configure      = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &cfg;

    PrivIoctl(adapter->fd, OPE_INT, &ioctl_cfg);
    PrivTaskDelay(1000);

    printf("Zigbee uart config ready\n");
    return 0;
}
#endif

static int E18NetworkModeConfig(struct Adapter *adapter)
{
    int ret = 0;
    int mode = -1;

    if (NULL == adapter) {
        return -1;
    }
    mode = E18HardwareModeGet();
    if(E18_AS_HEX_MODE == mode)
    {
        ret = AtCmdConfigAndCheck(adapter->agent, cmd_hex2at, "+OK");
        if(ret < 0) {
            printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,cmd_hex2at);
            ret = -1;
            goto out;
        }
    }
    

    switch (adapter->info->work_mode)
    {
        case TT_MODE1:
            ret = AtCmdConfigAndCheck(adapter->agent, cmd_mode_as_tt, "+OK");
            ret |= AtCmdConfigAndCheck(adapter->agent, cmd_ask_mode, "MODE=1");
            if(ret < 0) {
                printf("%s %d config net mode as transparent transmission failed!\n",__func__,__LINE__);
                ret = -1;
                goto out;
            }
            break;
        case STT_MODE2: 
            ret = AtCmdConfigAndCheck(adapter->agent, cmd_mode_as_stt, "+OK");
            ret |= AtCmdConfigAndCheck(adapter->agent, cmd_ask_mode, "MODE=2");
            if(ret < 0) {
                printf("%s %d config net mode as semi transparent transmission failed!\n",__func__,__LINE__);
                ret = -1;
                goto out;
            }
            break;
        case PROTOCOL_MODE3: 
            ret = AtCmdConfigAndCheck(adapter->agent, cmd_mode_as_protocol, "+OK");
            ret |= AtCmdConfigAndCheck(adapter->agent, cmd_ask_mode, "MODE=3");
            if(ret < 0) {
                printf("%s %d config net mode as protocol failed!\n",__func__,__LINE__);
                ret = -1;
                goto out;
            }
            break;
        default :
            ret = -1;
            break;
    }

out:
    if(E18_AS_HEX_MODE == mode){
        AtCmdConfigAndCheck(adapter->agent, cmd_exit, "+OK");
    }
    
    return ret;
}

static int E18NetRoleConfig(struct Adapter *adapter)
{
    int ret = 0;
    int mode = -1;


    if (NULL == adapter) {
        printf("%s %d adapter is null!\n",__func__,__LINE__);
        ret = -1;
        goto out;
    }
    mode = E18HardwareModeGet();
    if(E18_AS_HEX_MODE == mode)
    {
        ret = AtCmdConfigAndCheck(adapter->agent, cmd_hex2at, "+OK");
        if(ret < 0) {
            printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,cmd_hex2at);
            ret = -1;
            goto out;
        }
    }
    

    switch (adapter->net_role)
    {
        case COORDINATOR:
            ret = AtCmdConfigAndCheck(adapter->agent, cmd_role_as_c, "+OK");
            ret |= AtCmdConfigAndCheck(adapter->agent, cmd_ask_role, "DEV=C");
            if(ret < 0) {
                printf("%s %d config net role as coordinator failed!\n",__func__,__LINE__);
                ret = -1;
                goto out;
            }
            break;
        case ROUTER:
            ret = AtCmdConfigAndCheck(adapter->agent, cmd_role_as_r, "+OK");
            ret |= AtCmdConfigAndCheck(adapter->agent, cmd_role_as_r, "DEV=R");
            if(ret < 0) {
                printf("%s %d config net role as router failed!\n",__func__,__LINE__);
                ret = -1;
                goto out;
            }
            break;
        case END_DEVICE:
            ret = AtCmdConfigAndCheck(adapter->agent, cmd_role_as_e, "+OK");
            ret |= AtCmdConfigAndCheck(adapter->agent, cmd_ask_role, "DEV=E");
            if(ret < 0) {
                printf("%s %d config net role as device failed!\n",__func__,__LINE__);
                ret = -1;
                goto out;
            }
            break;
        default :
            ret = -1;
            break;
    }

out:
    if(E18_AS_HEX_MODE == mode) {
        AtCmdConfigAndCheck(adapter->agent, cmd_exit, "+OK");
    }

    return ret;
}

static int E18Open(struct Adapter *adapter)
{
    int ret = 0;
    int try_times = 5;
    int count = 0;

    if (NULL == adapter) {
        return -1;
    }
    /*step1: open e18 serial port*/
    ret = E18UartOpen(adapter);
    if (ret < 0) {
        printf("e18 setup failed.\n");
        return -1;
    }
     /*step2: init AT agent*/
    if (!adapter->agent) {
        char *agent_name = "zigbee_device";
        if (0 != InitATAgent(agent_name, adapter->fd, 512)) {
            printf("at agent init failed !\n");
            return -1;
        }
        ATAgentType at_agent = GetATAgent(agent_name);
        adapter->agent = at_agent;
    }

    AtSetReplyLrEnd(adapter->agent, 1);

try_again:
    while(try_times--){
        ret = E18NetRoleConfig(adapter);
        count++;
        if(ret < 0){
            printf("E18NetRoleConfig failed [%d] times.\n",count);
            continue;
        } else {
            break;
        }
    }
    
    if(ret < 0){
        printf("E18NetRoleConfig failed\n");
        return -1;
    }

    printf("Zigbee E18 open successful\n");
    return 0;
}

static int E18Close(struct Adapter *adapter)
{
    PrivClose(adapter->fd);
    return 0;
}

static int E18Ioctl(struct Adapter *adapter, int cmd, void *args)
{
    int ret = 0;
    switch (cmd)
    {
        case CONFIG_ZIGBEE_NET_ROLE: /* config E18 net role */
            ret = E18NetRoleConfig(adapter);
            break;
        case CONFIG_ZIGBEE_NET_MODE: /* config E18 network mode */
            ret = E18NetworkModeConfig(adapter);
            break;
        default:
            ret = -1;
            break;
    }

    return ret;
}

static int E18Join(struct Adapter *adapter, unsigned char *priv_net_group)
{
    int ret = 0;
    int mode = -1;

    mode = E18HardwareModeGet();
    if(E18_AS_HEX_MODE == mode)
    {
        ret = AtCmdConfigAndCheck(adapter->agent, cmd_hex2at, "+OK");
        if(ret < 0) {
            printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,cmd_hex2at);
            ret = -1;
            goto out;
        }
    }

    switch (adapter->net_role)
    {
        case COORDINATOR:
        case ROUTER:
        case END_DEVICE:
        /* config panid */
        ret = AtCmdConfigAndCheck(adapter->agent, cmd_set_panid, "+OK");
        ret |= AtCmdConfigAndCheck(adapter->agent, cmd_ask_panid, "PANID=A1B2");
        if(ret < 0) {
            printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,cmd_set_panid);
            ret = -1;
            goto out;
        }

        /* config channel*/
        ret = AtCmdConfigAndCheck(adapter->agent, cmd_set_ch, "+OK");
        if(ret < 0) {
            printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,cmd_set_ch);
            ret = -1;
            goto out;
        }
            break;
        default :
            ret = -1;
            break;
    }

    // switch (adapter->info->work_mode)
    // {
    //     case TT_MODE1:
    //         break;
    //     case STT_MODE2:
    //         /* need to do */
    //         ret = 0;
    //         break;
    //     case PROTOCOL_MODE3:
    //          /* need to do */
    //          ret = 0;
    //         break;
    //     default:
    //         ret = -1;
    //         break;

    // }
    if(!ret){
        if(E18_AS_HEX_MODE == mode) {
            ret = AtCmdConfigAndCheck(adapter->agent, cmd_exit, "+OK");
            if(ret < 0) {
                printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,cmd_exit);
                ret = -1;
            }
        } else {
            ret = -1;
        }
    }

out:
    return ret;
}

static int E18Send(struct Adapter *adapter, const void *buf, size_t len)
{
    int ret = 0;
    switch (adapter->info->work_mode)
    {
        case TT_MODE1:
            EntmSend(adapter->agent, buf, len);
            break;
        case STT_MODE2:
            /* need to do */
            ret = 0;
            break;
        case PROTOCOL_MODE3:
             /* need to do */
             ret = 0;
            break;
        default:
            ret = -1;
            break;

    }
    return 0;
}

static int E18Recv(struct Adapter *adapter, void *buf, size_t len)
{
    int ret = 0;
    switch (adapter->info->work_mode)
    {
        case TT_MODE1:
            if(!adapter->agent){
                PrivRead(adapter->fd, buf, len);
            } else {
               EntmRecv(adapter->agent, buf, len, 3);/* wait timeout 3000ms*/
            }
            break;
        case STT_MODE2:
            /* need to do */
            ret = 0;
            break;
        case PROTOCOL_MODE3:
             /* need to do */
             ret = 0;
            break;
        default:
            ret = -1;
            break;
    }
    
    return 0;
}

static int E18Quit(struct Adapter *adapter)
{

    return 0;
}

static const struct PrivProtocolDone E18_done = 
{
    .open = E18Open,
    .close = E18Close,
    .ioctl = E18Ioctl,
    .setup = NULL,
    .setdown = NULL,
    .setaddr = NULL,
    .setdns = NULL,
    .setdhcp = NULL,
    .ping = NULL,
    .netstat = NULL,
    .join = E18Join,
    .send = E18Send,
    .recv = E18Recv,
    .quit = E18Quit,
};

AdapterProductInfoType E18Attach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = malloc(sizeof(struct AdapterProductInfo));
    if (!product_info) {
        printf("E18Attach malloc product_info error\n");
        return NULL;
    }
    memset(product_info, 0, sizeof(struct AdapterProductInfo));
    
    strncpy(product_info->model_name, ADAPTER_ZIGBEE_E18,sizeof(product_info->model_name));
    product_info->model_done = (void *)&E18_done;

    return product_info;
}



