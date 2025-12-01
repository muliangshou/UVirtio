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
 * @file opcua_demo.c
 * @brief Demo for OpcUa function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.11.11
 */

#include <list.h>
#include <transform.h>
#include "board.h"
#include "open62541.h"
#include "ua_api.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define UA_URL_SIZE    100
#define UA_STACK_SIZE  4096
#define UA_TASK_PRIO   15

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
// to count test
static int test_cnt = 0;
static int fail_cnt = 0; // count failure times

char test_ua_ip[] = {192, 168, 250, 2};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void UaConnectTestTask(void* arg)
{
    struct netif net;
    UA_StatusCode ret;
    char url[UA_URL_SIZE];
    memset(url, 0, sizeof(url));

    UA_Client* client = UA_Client_new();

    if(client == NULL)
    {
        ua_error("ua: [%s] tcp client null\n", __func__);
        return;
    }

    UA_ClientConfig* config = UA_Client_getConfig(client);

    UA_ClientConfig_setDefault(config);

    snprintf(url, sizeof(url), "opc.tcp://%d.%d.%d.%d:4840",
             test_ua_ip[0], test_ua_ip[1], test_ua_ip[2], test_ua_ip[3]);

    ua_notice("ua connect cnt %d fail %d\n", test_cnt++, fail_cnt ++);
    ua_notice("ua connect uri: %.*s\n", strlen(url), url);

    ret = UA_Client_connect(client, url);

    if(ret != UA_STATUSCODE_GOOD)
    {
        ua_error("ua: [%s] connected failed %x\n", __func__, ret);
        UA_Client_delete(client);
        fail_cnt++;
        return;
    }

    ua_notice("ua connected ok!\n");
    UA_Client_delete(client);
}

static void UaConnectTest(int argc, char *argv[])
{
    if(argc == 2)
    {
        if(isdigit(argv[1][0]))
        {
            if(sscanf(argv[1], "%d.%d.%d.%d", &test_ua_ip[0], &test_ua_ip[1], &test_ua_ip[2], &test_ua_ip[3]) == EOF)
            {
                lw_notice("input wrong ip\n");
                return;
            }
        }
    }

    lwip_config_tcp(lwip_ipaddr, lwip_netmask, test_ua_ip);
    sys_thread_new("ua test", UaConnectTestTask, NULL, UA_STACK_SIZE, UA_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
                 UaConnect, UaConnectTest, Test Opc UA connection);

void UaBrowserObjectsTestTask(void* param)
{
    ua_notice("ua: [%s] start %d ...\n", __func__, test_cnt++);

    UA_Client* client = UA_Client_new();

    if(client == NULL)
    {
        ua_error("ua: [%s] tcp client NULL\n", __func__);
        return;
    }

    UA_ClientConfig* config = UA_Client_getConfig(client);
    UA_ClientConfig_setDefault(config);
    UA_StatusCode ret = UA_Client_connect(client, opc_server_url);

    if(ret != UA_STATUSCODE_GOOD)
    {
        ua_error("ua: [%s] connect failed %#x\n", __func__, ret);
        UA_Client_delete(client);
        return;
    }

    ua_notice("--- start read time ---\n", __func__);
    UaGetServerTime(client);

    ua_notice("--- get server info ---\n", __func__);
    UaTestBrowserObjects(client);

    /* Clean up */
    UA_Client_delete(client);    /* Disconnects the client internally */
}

static void* UaBrowserObjectsTest(int argc, char* argv[])
{
    if(argc == 2)
    {
        if(isdigit(argv[1][0]))
        {
            if(sscanf(argv[1], "%d.%d.%d.%d", &test_ua_ip[0], &test_ua_ip[1], &test_ua_ip[2], &test_ua_ip[3]) == EOF)
            {
                lw_notice("input wrong ip\n");
                return NULL;
            }
        }
    }

    lwip_config_tcp(lwip_ipaddr, lwip_netmask, test_ua_ip);
    sys_thread_new("ua object", UaBrowserObjectsTestTask, NULL, UA_STACK_SIZE, UA_TASK_PRIO);
    return NULL;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 UaObject, UaBrowserObjectsTest, UaObject [IP]);

void UaGetInfoTestTask(void* param)
{
    UA_Client* client = UA_Client_new();
    ua_notice("--- Get OPUCA objects ---\n", __func__);

    if(client == NULL)
    {
        ua_error("ua: [%s] tcp client null\n", __func__);
        return;
    }

    UA_ClientConfig* config = UA_Client_getConfig(client);
    UA_ClientConfig_setDefault(config);
    UA_StatusCode ret = UA_Client_connect(client, opc_server_url);

    if(ret != UA_STATUSCODE_GOOD)
    {
        ua_error("ua: [%s] connect failed %#x\n", __func__, ret);
        UA_Client_delete(client);
        return;
    }

    ua_notice("--- interactive server ---\n", __func__);
    UaTestInteractServer(client);

    UA_Client_delete(client);    /* Disconnects the client internally */
}

void* UaGetInfoTest(int argc, char* argv[])
{
    if(argc == 2)
    {
        if(isdigit(argv[1][0]))
        {
            if(sscanf(argv[1], "%d.%d.%d.%d", &test_ua_ip[0], &test_ua_ip[1], &test_ua_ip[2], &test_ua_ip[3]) == EOF)
            {
                lw_notice("input wrong ip\n");
                return NULL;
            }
        }
    }

    lwip_config_tcp(lwip_ipaddr, lwip_netmask, test_ua_ip);
    sys_thread_new("ua info", UaGetInfoTestTask, NULL, UA_STACK_SIZE, UA_TASK_PRIO);
    return NULL;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 UaInfo, UaGetInfoTest, UaInfo [IP]);

void UaAddNodesTask(void* param)
{
    UA_Client* client = UA_Client_new();
    ua_notice("ua: [%s] start ...\n", __func__);

    if(client == NULL)
    {
        ua_error("ua: [%s] client null\n", __func__);
        return;
    }

    UA_ClientConfig* config = UA_Client_getConfig(client);
    UA_ClientConfig_setDefault(config);
    UA_StatusCode ret = UA_Client_connect(client, opc_server_url);

    if(ret != UA_STATUSCODE_GOOD)
    {
        ua_print("ua: [%s] connect failed %#x\n", __func__, ret);
        UA_Client_delete(client);
        return;
    }

    ua_notice("--- add nodes ---\n", __func__);
    UaAddNodes(client);

    UA_Client_delete(client);    /* Disconnects the client internally */
}

void* UaAddNodesTest(int argc, char* argv[])
{
    if(argc == 2)
    {
        if(isdigit(argv[1][0]))
        {
            if(sscanf(argv[1], "%d.%d.%d.%d", &test_ua_ip[0], &test_ua_ip[1], &test_ua_ip[2], &test_ua_ip[3]) == EOF)
            {
                lw_notice("input wrong ip\n");
                return NULL;
            }
        }
    }

    lwip_config_tcp(lwip_ipaddr, lwip_netmask, test_ua_ip);
    sys_thread_new("ua add nodes", UaAddNodesTask, NULL, UA_STACK_SIZE, UA_TASK_PRIO);
    return NULL;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 UaAdd, UaAddNodesTest, UA Add Nodes);

