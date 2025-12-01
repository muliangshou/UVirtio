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
 * @file adapter_nbiot.c
 * @brief Implement the connection nbiot adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.06.25
 */
#include <transform.h>
#include <adapter.h>
#include <stdlib.h>

#ifdef ADAPTER_BC28
extern AdapterProductInfoType BC28Attach(struct Adapter *adapter);
#endif

static int AdapterNbiotRegister(struct Adapter *adapter)
{
    int ret = 0;

    strncpy(adapter->name, ADAPTER_NBIOT_NAME, NAME_NUM_MAX); 
 
    adapter->net_protocol = IP_PROTOCOL;
    adapter->net_role = CLIENT;

    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0) {
        printf("AdapterNbiot register error\n");
        return -1;
    }

    return ret;
}

int AdapterNbiotInit(void)
{
    int ret = 0;

    struct Adapter *adapter = PrivMalloc(sizeof(struct Adapter));
    if (!adapter) {
        printf("malloc adapter failed.\n");
        return -1;
    }
    memset(adapter, 0, sizeof(struct Adapter));
    ret = AdapterNbiotRegister(adapter);
    if (ret < 0) {
        printf("register nbiot adapter error\n");
        PrivFree(adapter);
        return -1;
    }
#ifdef ADAPTER_BC28
    AdapterProductInfoType product_info = BC28Attach(adapter);
    if (!product_info) {
        printf("bc28 attach error\n");
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
int opennb(void)
{
    int ret = 0;

    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_NBIOT_NAME);

#ifdef ADAPTER_BC28
    ret = AdapterDeviceOpen(adapter);
    if(ret < 0){
        printf("open adapter failed\n");
        return -1;
    }

#endif

     return 0;    
 }
//  SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, opennb, opennb, show adapter nb information);
int closenb(void)
{
    int ret = 0;

    struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_NBIOT_NAME);

#ifdef ADAPTER_BC28
    ret = AdapterDeviceClose(adapter);
    if(ret < 0){
        printf("open adapter failed\n");
        return -1;
    }

#endif

     return 0;    
 }
//  SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, closenb, closenb, show adapter nb information);
 
 int connectnb(int argc, char *argv[])
 {
     const char *send_msg = argv[1];
     int ret = 0;

     struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_NBIOT_NAME);

    
     ret = AdapterDeviceConnect(adapter, 1, "101.68.82.219","9898",1);
     if(ret < 0){
         printf(" adapter send failed\n");
         return -1;
     }
    


     return 0;    
 }
//  SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(2)|SHELL_CMD_DISABLE_RETURN, connectnb, connectnb, show adapter nb information);

 int sendnb(int argc, char *argv[])
 {
     const char *send_msg = argv[1];
     int msg_len = atoi(argv[2]);
     int ret = 0;

     struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_NBIOT_NAME);

     printf("send argv1 %s len = %d\n",argv[1],msg_len);
     ret = AdapterDeviceSend(adapter, send_msg, msg_len);
     if(ret < 0){
         printf(" adapter send failed\n");
         return -1;
     }
     printf("nb send msg %s\n", send_msg);


     return 0;    
 }
//  SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_PARAM_NUM(2)|SHELL_CMD_DISABLE_RETURN, sendnb, sendnb, show adapter nb information);

 int recvnb(void)
 {
     char recv_msg[128];
     struct Adapter* adapter =  AdapterDeviceFindByName(ADAPTER_NBIOT_NAME);
     memset(recv_msg,0,128);
     AdapterDeviceRecv(adapter, recv_msg, 128);
     PrivTaskDelay(2000);
     printf("nb recv msg %s\n", recv_msg);

     return 0;    
 }
//  SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, recvnb, recvnb, show adapter nb information);

