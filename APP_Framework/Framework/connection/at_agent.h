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
 * @file xs_adapter_at_agent.h
 * @brief AT proxy, auto receive AT reply and transparency data
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.04.22
 */

#ifndef XS_ADAPTER_AT_CLIENT_H
#define XS_ADAPTER_AT_CLIENT_H

#include <list.h>
#include <transform.h>
#include <stdio.h>
#include <sys/types.h>

#define REPLY_TIME_OUT 10

enum ReceiveMode
{
    DEFAULT_MODE = 0,
    ENTM_MODE = 1,
    AT_MODE = 2,
};

struct ATReply
{
    char *reply_buffer;
    uint32 reply_max_len;
    uint32 reply_len;
};
typedef struct ATReply *ATReplyType;

struct ATAgent
{
    char agent_name[64];
    int fd;
    int read_len;   ///< control data length of wanting reading

    char *maintain_buffer;
    uint32 maintain_len;
    uint32 maintain_max;

#ifdef ADD_NUTTX_FETURES   
    pthread_mutex_t lock;
#else
    int lock;
#endif

    ATReplyType reply;
    char reply_lr_end;
    char reply_end_last_char;
    char reply_end_char;
    uint32 reply_char_num;
    int rsp_sem;

    pthread_t at_handler;

    #define ENTM_RECV_MAX 256
    char entm_recv_buf[ENTM_RECV_MAX];
    uint32 entm_recv_len;
    enum ReceiveMode receive_mode;
    int entm_rx_notice;
};
typedef struct ATAgent *ATAgentType;

int EntmSend(ATAgentType agent, const char *data, int len);
int EntmRecv(ATAgentType agent, char *rev_buffer, int buffer_len, int timeout_s);
char *GetReplyText(ATReplyType reply);
int AtSetReplyEndChar(ATAgentType agent, char last_ch, char end_ch);
int AtSetReplyCharNum(ATAgentType agent, unsigned int num);
int AtSetReplyLrEnd(ATAgentType agent, char enable);
ATReplyType CreateATReply(uint32 reply_max_len);
unsigned int IpTint(char *ipstr);
void SwapStr(char *str, int begin, int end);
char* IpTstr(unsigned int ipint);
ATAgentType GetATAgent(const char *agent_name);
int InitATAgent(const char *agent_name, int fd, uint32 maintain_max);
int DeleteATAgent(ATAgentType agent);
int ParseATReply(char* str, const char *format, ...);
void DeleteATReply(ATReplyType reply);
int ATOrderSend(ATAgentType agent, uint32_t timeout_s, ATReplyType reply, const char *cmd_expr, ...);
int AtCmdConfigAndCheck(ATAgentType agent, char *cmd, char *check);

#endif