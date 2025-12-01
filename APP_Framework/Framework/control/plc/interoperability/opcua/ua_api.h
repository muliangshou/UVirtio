/*
* Copyright (c) 2021 AIIT XUOS Lab
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
 * @file ua_api.h
 * @brief API for OpcUa function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.11.11
 */

#ifndef __UA_API_H__
#define __UA_API_H__

#include "open62541.h"

#define UA_DEV_IP_LEN 48
#define UA_NODE_LEN 32

enum UaAction_e
{
    UA_ACT_ATTR,
    UA_ACT_OBJ,
};

typedef struct UaParam
{
    enum UaAction_e act;
    UA_NodeId ua_id;
    char ua_remote_ip[UA_DEV_IP_LEN];
    char ua_node[UA_NODE_LEN];
    UA_Client *client;
}UaParamType;

#define ua_print //KPrintf
#define ua_notice KPrintf
#define ua_error KPrintf

extern const char *opc_server_url;
extern char test_ua_ip[];

int UaServerRun(void);
void UaBrowserNodes(UA_Client *client);
void UaBrowserNodeId(UA_Client *client, UA_NodeId id);
void UaGetServerTime(UA_Client *client);
void UaAddNodes(UA_Client *client);

int UaDevOpen(void *dev); // open and connect PLC device
void UaDevClose(void* dev); // close and disconnect PLC device
int UaDevRead(void* dev, void *buf, size_t len); // read data from PLC
int UaDevWrite(void* dev, const void *buf, size_t len); // write data from PLC
int UaDevIoctl(void* dev, int cmd, void *arg); // send control command to PLC

char *UaGetNodeIdString(UA_NodeId *node_id);
void UaReadNodeValue(UA_Client *client, UA_NodeId id, UA_Int32 *value);
void UaWriteNodeValue(UA_Client *client, UA_NodeId id, char* value);
UA_StatusCode UaReadArrayValue(UA_Client *client, int array_size, UA_ReadValueId *array);
int UaGetNodeIdArray(UA_Client* client, UA_NodeId id, int array_size, int *id_array);

void UaTestBrowserObjects(UA_Client *client);
int UaTestInteractServer(UA_Client *client);

#endif
