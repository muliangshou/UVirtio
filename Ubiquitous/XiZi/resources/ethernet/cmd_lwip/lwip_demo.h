/*
* Copyright (c) 2022 AIIT XUOS Lab
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
* @file lwip_demo.h
* @brief lwip demo relative struct and definition
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-21
*/

#ifndef __LWIP_DEMO_H__
#define __LWIP_DEMO_H__

typedef struct LwipTcpSocketStruct
{
    char ip[6];
    uint16_t port;
    char *buf;
}LwipTcpSocketParamType;

#define LWIP_TEST_MSG_SIZE 128

#define LWIP_TEST_STACK_SIZE 4096
#define LWIP_TEST_TASK_PRIO 20

#endif /* __LWIP_DEMO_H__ */

