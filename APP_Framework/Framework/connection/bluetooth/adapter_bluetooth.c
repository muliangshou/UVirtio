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
 * @file adapter_bluetooth.c
 * @brief Implement the connection bluetooth adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.06.25
 */

#include <adapter.h>

#ifdef ADAPTER_HC08
extern AdapterProductInfoType Hc08Attach(struct Adapter *adapter);
#endif

static int AdapterBlueToothRegister(struct Adapter *adapter)
{
    int ret = 0;

    strncpy(adapter->name, ADAPTER_BLUETOOTH_NAME, NAME_NUM_MAX);

    adapter->net_protocol = IP_PROTOCOL;
    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0) {
        printf("AdapterBlueToothRegister register error\n");
        return -1;
    }

    return ret;
}

int AdapterBlueToothInit(void)
{
    int ret = 0;

    struct Adapter *adapter = PrivMalloc(sizeof(struct Adapter));
    if (!adapter) {
        PrivFree(adapter);
        return -1;
    }

    memset(adapter, 0, sizeof(struct Adapter));

    ret = AdapterBlueToothRegister(adapter);
    if (ret < 0) {
        printf("AdapterBlueToothInit register BT adapter error\n");
        PrivFree(adapter);
        return -1;
    }

#ifdef ADAPTER_HC08
    AdapterProductInfoType product_info = Hc08Attach(adapter);
    if (!product_info) {
        printf("AdapterBlueToothInit hc08 attach error\n");
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
int AdapterBlueToothTest(void)
{
    const char *bluetooth_msg = "BT Adapter Test";
    char bluetooth_recv_msg[128];
    int len;
    int baud_rate = BAUD_RATE_115200;

    struct Adapter *adapter =  AdapterDeviceFindByName(ADAPTER_BLUETOOTH_NAME);

#ifdef ADAPTER_HC08
    AdapterDeviceOpen(adapter);
    AdapterDeviceControl(adapter, OPE_INT, &baud_rate);

    len = strlen(bluetooth_msg);

    while (1) {
        AdapterDeviceRecv(adapter, bluetooth_recv_msg, 128);
        printf("bluetooth_recv_msg %s\n", bluetooth_recv_msg);
        AdapterDeviceSend(adapter, bluetooth_msg, len);
        printf("send %s after recv\n", bluetooth_msg);
        PrivTaskDelay(100);
        memset(bluetooth_recv_msg, 0, 128);
    }

#endif

    return 0;    
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, AdapterBlueToothTest, AdapterBlueToothTest, show adapter BT information);
