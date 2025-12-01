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
 * @file adapter_wifiiiii.c
 * @brief Implement the connection wifi adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.07.25
 */

#include <adapter.h>
#include "adapter_wifi.h"
#include <bus_pin.h>

#ifdef ADAPTER_HFA21_WIFI
extern AdapterProductInfoType Hfa21WifiAttach(struct Adapter *adapter);
#endif
#ifdef ADAPTER_ESP07S_WIFI
extern AdapterProductInfoType Esp07sWifiAttach(struct Adapter *adapter);
#endif

static int AdapterWifiRegister(struct Adapter *adapter)
{
    int ret = 0;

    strncpy(adapter->name, ADAPTER_WIFI_NAME, NAME_NUM_MAX);

    adapter->net_protocol = IP_PROTOCOL;
    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0) {
        printf("AdapterWifi register error\n");
        return -1;
    }

    return ret;
}

int AdapterWifiInit(void)
{
    int ret = 0;

    struct Adapter *adapter = PrivMalloc(sizeof(struct Adapter));
    if (!adapter) {
        printf("AdapterWifiInit malloc error\n");
        PrivFree(adapter);
        return -1;
    }

    memset(adapter, 0, sizeof(struct Adapter));

    ret = AdapterWifiRegister(adapter);
    if (ret < 0) {
        printf("AdapterWifiInit register wifi adapter error\n");
        PrivFree(adapter);
        return -1;
    }

#ifdef ADAPTER_HFA21_WIFI
    AdapterProductInfoType product_info = Hfa21WifiAttach(adapter);
    if (!product_info) {
        printf("AdapterWifiInit hfa21 attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;

#endif
#ifdef ADAPTER_ESP07S_WIFI
    AdapterProductInfoType product_info = Esp07sWifiAttach(adapter);
    if (!product_info) {
        printf("AdapterWifiInit ESP07S attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;

#endif

    return ret;
}

/******************wifi TEST*********************/
int AdapterwifiTest(void)
{
    char cmd[64];
    int baud_rate = BAUD_RATE_57600;

    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);


#ifdef ADAPTER_HFA21_DRIVER_EXT_PORT
    // static BusType ch438_pin;
    // ch438_pin = PinBusInitGet();
	// struct PinParam pin_cfg;	
	// int ret = 0;

	// struct BusConfigureInfo configure_info;
	// configure_info.configure_cmd = OPE_CFG;
	// configure_info.private_data = (void *)&pin_cfg;

    // pin_cfg.cmd = GPIO_CONFIG_MODE;
    // pin_cfg.pin = 22;
    // pin_cfg.mode = GPIO_CFG_OUTPUT;

	// ret = BusDrvConfigure(ch438_pin->owner_driver, &configure_info);

    // struct PinStat pin_stat;
	// struct BusBlockWriteParam write_param;
	// struct BusBlockReadParam read_param;
	// write_param.buffer = (void *)&pin_stat;
	
	// pin_stat.val = GPIO_HIGH;

    // pin_stat.pin = 22;
    // BusDevWriteData(ch438_pin->owner_haldev, &write_param);

    int pin_fd;
    pin_fd = PrivOpen("/dev/pin_dev", O_RDWR);

    struct PinParam pin_param;
    pin_param.cmd = GPIO_CONFIG_MODE;
    pin_param.mode = GPIO_CFG_OUTPUT;
    pin_param.pin = 22;

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = PIN_TYPE;
    ioctl_cfg.args = &pin_param;
    PrivIoctl(pin_fd, OPE_CFG, &ioctl_cfg);

    struct PinStat pin_stat;
    pin_stat.pin = 52;
    pin_stat.val = GPIO_HIGH;
    PrivWrite(pin_fd, &pin_stat, 1);

    PrivClose(pin_fd);
#endif


    AdapterDeviceOpen(adapter);
    // AdapterDeviceControl(adapter, OPE_INT, &baud_rate);

    AdapterDeviceSetUp(adapter);
    AdapterDeviceSetAddr(adapter, "192.168.64.253", "192.168.66.1", "255.255.252.0");
    AdapterDevicePing(adapter, "36.152.44.95");
    AdapterDeviceNetstat(adapter);

    const char *ip = "192.168.64.60";
    const char *port = "12345";
    enum NetRoleType net_role = CLIENT;
    enum IpType ip_type = IPV4;
    AdapterDeviceConnect(adapter, net_role, ip, port, ip_type);

    const char *wifi_msg = "LiuKai Test";
    int len = strlen(wifi_msg);
    for(int i = 0;i < 10; ++i) {
        AdapterDeviceSend(adapter, wifi_msg, len);
        PrivTaskDelay(4000);
    }

    char wifi_recv_msg[128];
    while (1) {
        AdapterDeviceRecv(adapter, wifi_recv_msg, 128);
    }
    
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, AdapterwifiTest, AdapterwifiTest, show adapter wifi information);

int wifiopen(void)
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);

    AdapterDeviceOpen(adapter);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, wifiopen, wifiopen, open adapter wifi );
int wificlose(void)
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);

    AdapterDeviceClose(adapter);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, wificlose, wificlose, close adapter wifi );

int wifisetup(int argc, char *argv[])
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);
    struct WifiParam param;
    memset(&param,0,sizeof(struct WifiParam));
    strncpy(param.wifi_ssid, argv[1], strlen(argv[1]));
    strncpy(param.wifi_pwd, argv[2], strlen(argv[2]));

    adapter->adapter_param = &param;

    AdapterDeviceSetUp(adapter);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(3)|SHELL_CMD_DISABLE_RETURN, wifisetup, wifisetup, setup adapter wifi );
int wifiaddrset(int argc, char *argv[])
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);
    char *ip = argv[1];
    char *gateway = argv[2];
    char *netmask = argv[3];

    AdapterDeviceSetAddr(adapter, ip, gateway, netmask);
    AdapterDevicePing(adapter, "36.152.44.95");///< ping www.baidu.com
    AdapterDeviceNetstat(adapter);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(4)|SHELL_CMD_DISABLE_RETURN, wifiaddrset, wifiaddrset, addrset adapter wifi);

int wifiping(int argc, char *argv[])
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);
    printf("ping %s\n",argv[1]);
    AdapterDevicePing(adapter, argv[1]);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(3), wifiping, wifiping, wifiping adapter );

int wificonnect(int argc, char *argv[])
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);
    char *ip = argv[1];
    char *port = argv[2];
    enum NetRoleType net_role = CLIENT;
    enum IpType ip_type = IPV4;

    if(0 == strncmp("tcp",argv[3],strlen("tcp"))) {
        adapter->socket.protocal = SOCKET_PROTOCOL_TCP;
    }

    if(0 == strncmp("udp",argv[3],strlen("udp"))) {
        adapter->socket.protocal = SOCKET_PROTOCOL_UDP;
    }

    AdapterDeviceConnect(adapter, net_role, ip, port, ip_type);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(4)|SHELL_CMD_DISABLE_RETURN, wificonnect, wificonnect, wificonnect adapter);
int wifisend(int argc, char *argv[])
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);

    const char *wifi_msg = argv[1];
    int len = strlen(wifi_msg);
    for(int i = 0;i < 10; ++i) {
        AdapterDeviceSend(adapter, wifi_msg, len);
        PrivTaskDelay(1000);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(3)|SHELL_CMD_DISABLE_RETURN, wifisend, wifisend, wifisend adapter wifi information);
int wifirecv(int argc, char *argv[])
{
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_WIFI_NAME);

    char wifi_recv_msg[128];
    while (1) {
        AdapterDeviceRecv(adapter, wifi_recv_msg, 128);
        PrivTaskDelay(1000);
        printf("wifi recv [%s]\n",wifi_recv_msg);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(3)|SHELL_CMD_DISABLE_RETURN, wifirecv, wifirecv, wifirecv adapter wifi information);
