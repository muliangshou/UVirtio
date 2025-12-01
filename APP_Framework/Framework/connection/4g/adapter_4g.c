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
 * @file adapter_4G.c
 * @brief Implement the connection 4G adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.06.25
 */

#include <adapter.h>

#ifdef ADAPTER_EC200T
extern AdapterProductInfoType Ec200tAttach(struct Adapter *adapter);
#endif

static int Adapter4GRegister(struct Adapter *adapter)
{
    int ret = 0;

    strncpy(adapter->name, ADAPTER_4G_NAME, NAME_NUM_MAX);

    adapter->net_protocol = IP_PROTOCOL;
    adapter->net_role = CLIENT;
    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0) {
        printf("Adapter4G register error\n");
        return -1;
    }

    return ret;
}

int Adapter4GInit(void)
{
    int ret = 0;

    struct Adapter *adapter = PrivMalloc(sizeof(struct Adapter));
    if (!adapter) {
        PrivFree(adapter);
        return -1;
    }

    memset(adapter, 0, sizeof(struct Adapter));

    ret = Adapter4GRegister(adapter);
    if (ret < 0) {
        printf("Adapter4GInit register 4G adapter error\n");
        PrivFree(adapter);
        return -1;
    }

#ifdef ADAPTER_EC200T
    AdapterProductInfoType product_info = Ec200tAttach(adapter);
    if (!product_info) {
        printf("Adapter4GInit ec200t attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;

#endif

    return ret;
}

/******************4G TEST*********************/
int Adapter4GTest(void)
{
    const char *send_msg = "Adapter_4G Test";
    char recv_msg[256] = {0};
    int baud_rate = BAUD_RATE_115200;

    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_4G_NAME);

#ifdef ADAPTER_EC200T
    //Using Hang Xiao server to test 4G Socket connection
    uint8 server_addr[64] = "120.76.100.197";
    uint8 server_port[64] = "10002";

    adapter->socket.socket_id = 0;

    AdapterDeviceOpen(adapter);
    AdapterDeviceControl(adapter, OPE_INT, &baud_rate);

    AdapterDeviceConnect(adapter, CLIENT, server_addr, server_port, IPV4);

    while (1) {
        AdapterDeviceSend(adapter, send_msg, strlen(send_msg));
        AdapterDeviceRecv(adapter, recv_msg, 256);
        printf("4G recv msg %s\n", recv_msg);
        memset(recv_msg, 0, 256);
    }
#endif

    return 0;    
}
// SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, Adapter4GTest, Adapter4GTest, show adapter 4G information);
