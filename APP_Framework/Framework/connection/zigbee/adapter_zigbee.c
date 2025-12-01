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
 * @file adapter_zigbee.c
 * @brief Implement the connection zigbee adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.09.15
 */

#include <adapter.h>
#include "adapter_zigbee.h"


#ifdef ADAPTER_E18
extern AdapterProductInfoType E18Attach(struct Adapter *adapter);
#endif

static int AdapterZigbeeRegister(struct Adapter *adapter)
{
    int ret = 0;

    strncpy(adapter->name, ADAPTER_ZIGBEE_NAME, NAME_NUM_MAX); 

 /* config adapter  net protocol as network mode  */ 
    adapter->net_protocol = PRIVATE_PROTOCOL;

 /* config adapter  net role as coordinator/router/end_device */
#ifdef AS_COORDINATOR_ROLE
    adapter->net_role = COORDINATOR;
#endif
#ifdef AS_ROUTER_ROLE
    adapter->net_role = ROUTER;
#endif
#ifdef AS_END_DEVICE_ROLE
    adapter->net_role = END_DEVICE;
#endif

    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0) {
        printf("AdapterZigbee register error\n");
        return -1;
    }

    return ret;
}

int AdapterZigbeeInit(void)
{
    int ret = 0;

    struct Adapter *adapter = malloc(sizeof(struct Adapter));
    if (!adapter) {
        printf("malloc adapter failed.\n");
        return -1;
    }
    memset(adapter, 0, sizeof(struct Adapter));
    ret = AdapterZigbeeRegister(adapter);
    if (ret < 0) {
        printf("AdapterZigbeeRegister register zigbee adapter error\n");
        PrivFree(adapter);
        return -1;
    }
#ifdef ADAPTER_E18
    AdapterProductInfoType product_info = E18Attach(adapter);
    if (!product_info) {
        printf("AdapterZigbeeRegister e18 attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;
#endif

    return ret;
}

/******************TEST*********************/
int openzigbee(void)
{
    int ret = 0;

    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_ZIGBEE_NAME);

#ifdef ADAPTER_E18
    ret = AdapterDeviceOpen(adapter);
    if(ret < 0){
        printf("open adapter failed\n");
        return -1;
    }

    adapter->info->work_mode = 1;
    ret = AdapterDeviceControl(adapter, CONFIG_ZIGBEE_NET_MODE,NULL);
    if(ret < 0){
        printf("control adapter failed\n");
        return -1;
    }
    ret = AdapterDeviceJoin(adapter, NULL);
    if(ret < 0){
        printf("join adapter failed\n");
        return -1;
    }
#endif

    return 0;    
}

#ifdef ADD_XIZI_FETURES
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, openzigbee, openzigbee, show adapter zigbee information);
#endif

int sendzigbee(int argc, char *argv[])
{
    const char *send_msg = argv[1];
    int ret = 0;

    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_ZIGBEE_NAME);

    printf("send argv1 %s\n",argv[1]);
    ret = AdapterDeviceSend(adapter, send_msg, strlen(send_msg));
    if(ret < 0){
        printf(" adapter send failed\n");
        return -1;
    }
    printf("zigbee send msg %s\n", send_msg);
    

    return 0;    
}
#ifdef ADD_XIZI_FETURES
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(2)|SHELL_CMD_DISABLE_RETURN, sendzigbee, sendzigbee, show adapter zigbee information);
#endif

int recvzigbee(void)
{
    char recv_msg[128];
    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_ZIGBEE_NAME);
    memset(recv_msg,0,128);
    AdapterDeviceRecv(adapter, recv_msg, 128);
    PrivTaskDelay(2000);
    printf("zigbee recv msg %s\n", recv_msg);
    
    return 0;    
}
#ifdef ADD_XIZI_FETURES
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, recvzigbee, recvzigbee, show adapter zigbee information);
#endif

