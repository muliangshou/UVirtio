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
 * @file adapter_ethernet.c
 * @brief Implement the connection ethernet adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.10.15
 */

#include <adapter.h>

#ifdef ADAPTER_HFA21_ETHERNET
extern AdapterProductInfoType Hfa21EthernetAttach(struct Adapter *adapter);
#endif

static int AdapterEthernetRegister(struct Adapter *adapter)
{
    int ret = 0;

    strncpy(adapter->name, ADAPTER_ETHERNET_NAME, NAME_NUM_MAX);

    adapter->net_protocol = IP_PROTOCOL;
    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0) {
        printf("AdapterEthernet register error\n");
        return -1;
    }

    return ret;
}

int AdapterEthernetInit(void)
{
    int ret = 0;

    struct Adapter *adapter = PrivMalloc(sizeof(struct Adapter));
    if (!adapter) {
        printf("AdapterEthernetInit malloc error\n");
        PrivFree(adapter);
        return -1;
    }

    memset(adapter, 0, sizeof(struct Adapter));

    ret = AdapterEthernetRegister(adapter);
    if (ret < 0) {
        printf("AdapterEthernetInit register ethernet adapter error\n");
        PrivFree(adapter);
        return -1;
    }

#ifdef ADAPTER_HFA21_ETHERNET
    AdapterProductInfoType product_info = Hfa21EthernetAttach(adapter);
    if (!product_info) {
        printf("AdapterEthernetInit hfa21 attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;

#endif

    return ret;
}

/******************ethernet TEST*********************/
int AdapterEthernetTest(void)
{
    int baud_rate = BAUD_RATE_57600;

    struct Adapter *adapter =  AdapterDeviceFindByName(ADAPTER_ETHERNET_NAME);

#ifdef ADAPTER_HFA21_ETHERNET

    char ethernet_recv_msg[128] = {0};
    int i, len = 0;
    const char *ethernet_msg = "Adapter Ethernet Test";

    AdapterDeviceOpen(adapter);
    AdapterDeviceControl(adapter, OPE_INT, &baud_rate);

    AdapterDeviceSetUp(adapter);
    
    const char *ip = "10.10.100.50";
    const char *port = "12345";
    enum NetRoleType net_role = CLIENT;//SERVER
    enum IpType ip_type = IPV4;
    AdapterDeviceConnect(adapter, net_role, ip, port, ip_type);

    printf("ready to test data transfer\n");

    len = strlen(ethernet_msg);
    for (i = 0;i < 10; i ++) {
        printf("AdapterEthernetTest send %s\n", ethernet_msg);
        AdapterDeviceSend(adapter, ethernet_msg, len);
        PrivTaskDelay(4000);
    }

    while (1) {
        AdapterDeviceRecv(adapter, ethernet_recv_msg, 128);
        printf("AdapterEthernetTest recv %s\n", ethernet_recv_msg);
        memset(ethernet_recv_msg, 0, 128);
    }
#endif
    
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, AdapterEthernetTest, AdapterEthernetTest, show adapter ethernet information);
