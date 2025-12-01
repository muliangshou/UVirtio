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
 * @file ua_test.c
 * @brief Test for OpcUa function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.11.11
 */

#include <stdlib.h>
#include "open62541.h"
#include "ua_api.h"

//for target NODEID
#define UA_TEST_BROWSER_NODEID UA_NODEID_STRING(3, "ServerInterfaces")
#define UA_TEST_BROWSER_NODEID1 UA_NODEID_NUMERIC(4, 1)
#define UA_TEST_WRITE_NODEID UA_NODEID_NUMERIC(4, 5)

#define UA_TEST_NODE_ARRAY_NUM 10

static UA_StatusCode UaTestReadArrayValue(UA_Client *client, UA_NodeId id)
{
    int i;
    int array_size = 0;
    int test_id[UA_TEST_NODE_ARRAY_NUM];
    UA_ReadValueId test_array[UA_TEST_NODE_ARRAY_NUM];

    for(i = 0; i < UA_TEST_NODE_ARRAY_NUM; i++)
    {
        UA_ReadValueId_init(&test_array[i]);
        test_array[i].attributeId = UA_ATTRIBUTEID_VALUE;
    }

    array_size = UaGetNodeIdArray(client, id, UA_TEST_NODE_ARRAY_NUM, test_id);

    for(i = 0; i < array_size; i++)
    {
        test_array[i].nodeId = UA_NODEID_NUMERIC(id.namespaceIndex, test_id[i]);
    }

    return UaReadArrayValue(client, array_size, test_array);
}

void UaTestBrowserObjects(UA_Client *client)
{
    UA_NodeId test_id;
    UaBrowserNodeId(client, UA_TEST_BROWSER_NODEID);
    UaBrowserNodeId(client, UA_TEST_BROWSER_NODEID1);
    test_id = UA_TEST_BROWSER_NODEID1;
    ua_notice("Show values in %s:\n", UaGetNodeIdString(&test_id));
    UaTestReadArrayValue(client, test_id);
    return;
}

static void UaTestWriteNodeValue(UA_Client *client)
{
    UA_Int32 value;
    char val_str[UA_NODE_LEN];
    UA_NodeId id = UA_TEST_WRITE_NODEID;

    ua_notice("--- Test write %s ---\n", UaGetNodeIdString(&id));
    UaReadNodeValue(client, id, &value);
    UaWriteNodeValue(client, id, itoa(value + 1, val_str, 10));
    UaReadNodeValue(client, id, &value);
    ua_notice("\n");
}

int UaTestInteractServer(UA_Client *client)
{
    UaGetServerTime(client);
    UaTestBrowserObjects(client);
    UaTestWriteNodeValue(client);
    return EXIT_SUCCESS;
}

