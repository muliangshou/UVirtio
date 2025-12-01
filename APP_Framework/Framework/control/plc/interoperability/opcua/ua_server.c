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
 * @file ua_server.c
 * @brief Server for OpcUa function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.11.11
 */

#include "open62541.h"

UA_Boolean ua_server_flag = true;

int UaRunServer(void)
{
    UA_StatusCode ret;

    UA_Server *server = UA_Server_new();

    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    ret = UA_Server_run(server, &ua_server_flag);

    UA_Server_delete(server);

    return ret;
}
