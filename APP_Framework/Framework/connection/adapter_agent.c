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
 * @file xs_adapterAT_client.c
 * @brief AT proxy, auto receive AT reply and transparency data
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.04.22
 */


#include <at_agent.h>
#include <adapter.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifdef ADD_XIZI_FETURES
# include <user_api.h>
#endif

#define AT_CMD_MAX_LEN 128
#define AT_AGENT_MAX 2
static char send_buf[AT_CMD_MAX_LEN];
static uint32_t last_cmd_len = 0;

static struct ATAgent at_agent_table[AT_AGENT_MAX] = {0};

unsigned int IpTint(char *ipstr)
{
    if (ipstr == NULL)
        return 0;

    char *token;
    unsigned int i = 3, total = 0, cur;

    token = strtok(ipstr, ".");

    while (token != NULL) {
        cur = atoi(token);
        if (cur >= 0 && cur <= 255) {
            total += cur * pow(256, i);
        }
        i--;
        token = strtok(NULL, ".");
    }

    return total;
}

void SwapStr(char *str, int begin, int end)
{
    int i, j;

    for (i = begin, j = end; i <= j; i++, j--) {
        if (str[i] != str[j]) {
            str[i] = str[i] ^ str[j];
            str[j] = str[i] ^ str[j];
            str[i] = str[i] ^ str[j];
        }
    }
}

char *IpTstr(unsigned int ipint)
{
    int LEN = 16;
    char *new = (char *)PrivMalloc(LEN);
    memset(new, '\0', LEN);
    new[0] = '.';
    char token[4];
    int bt, ed, len, cur;

    while (ipint) {
        cur = ipint % 256;
        sprintf(token, "%d", cur);
        strcat(new, token);
        ipint /= 256;
        if (ipint)
            strcat(new, ".");
    }

    len = strlen(new);
    SwapStr(new, 0, len - 1);

    for (bt = ed = 0; ed < len;) {
        while (ed < len && new[ed] != '.') {
            ed++;
        }
        SwapStr(new, bt, ed - 1);
        ed += 1;
        bt = ed;
    }

    new[len - 1] = '\0';

    return new;
}

int ParseATReply(char *str, const char *format, ...)
{
    va_list params;
    int counts = 0;

    va_start(params, format);
    counts = vsscanf(str, format, params);
    va_end(params);

    return counts;
}

void ATSprintf(int fd, const char *format, va_list params)
{
    last_cmd_len = vsnprintf(send_buf, sizeof(send_buf), format, params);
    printf("AT send %s len %u\n",send_buf, last_cmd_len);
	PrivWrite(fd, send_buf, last_cmd_len);
}

int ATOrderSend(ATAgentType agent, uint32_t timeout_s, ATReplyType reply, const char *cmd_expr, ...)
{
    if (agent == NULL) {
        printf("ATAgent is null");
        return -1;
    }

    struct timespec abstime;

    abstime.tv_sec = timeout_s;

    PrivMutexObtain(&agent->lock); 
    agent->receive_mode = AT_MODE;

    memset(agent->maintain_buffer, 0x00, agent->maintain_max);
    agent->maintain_len = 0;

    memset(agent->entm_recv_buf, 0, ENTM_RECV_MAX);
    agent->entm_recv_len = 0;

    va_list params;
    uint32_t cmd_size = 0;
    uint32_t result = 0;
    const char *cmd = NULL;

    agent->reply = reply;
    PrivMutexAbandon(&agent->lock); 
    
    if(agent->reply != NULL) {
        PrivMutexObtain(&agent->lock);
        reply->reply_len = 0;
        va_start(params, cmd_expr);
        ATSprintf(agent->fd, cmd_expr, params);
        va_end(params);
        PrivMutexAbandon(&agent->lock);
        if (PrivSemaphoreObtainWait(&agent->rsp_sem, &abstime) != 0) {
            printf("take sem %d timeout\n",agent->rsp_sem);
            result = -2;
            goto __out;
        }
    } else {
        PrivMutexObtain(&agent->lock);
        va_start(params, cmd_expr);
        ATSprintf(agent->fd, cmd_expr, params);
        va_end(params);
        PrivMutexAbandon(&agent->lock);
        goto __out;
    }

__out:
    // agent->reply = NULL;
    return result;
}

int AtCmdConfigAndCheck(ATAgentType agent, char *cmd, char *check)
{
    int ret = 0;
    char *result = NULL;
    if (NULL == agent || NULL == cmd || NULL == check ) {
        return -1;
    }

    ATReplyType reply = CreateATReply(256);
    if (NULL == reply) {
        printf("%s %d at_create_resp failed!\n",__func__,__LINE__);
        ret = -1;
        goto __exit;
    }

    ret = ATOrderSend(agent, REPLY_TIME_OUT, reply, cmd);
    if(ret < 0){
        printf("%s %d ATOrderSend failed.\n",__func__,__LINE__);
        ret = -1;
        goto __exit;
    }

    //PrivTaskDelay(3000);

    result = GetReplyText(reply);
    if (!result) {
        printf("%s %n get reply failed.\n",__func__,__LINE__);
        ret = -1;
        goto __exit;
    }
    printf("[reply result :\n");
    printf("%s]\n", result);
    if(!strstr(result, check)) {
        printf("%s %d check[%s] reply[%s] failed.\n",__func__,__LINE__,check,result);
        ret = -1;
        goto __exit;
    }

__exit:
    DeleteATReply(reply);
    return ret;
}

char *GetReplyText(ATReplyType reply)
{
    return reply->reply_buffer;
}

int AtSetReplyLrEnd(ATAgentType agent, char enable)
{  
    if (!agent) {
        return -1;
    }

    agent->reply_lr_end = enable;

    return 0;
}

int AtSetReplyEndChar(ATAgentType agent, char last_ch, char end_ch)
{  
    if (!agent) {
        return -1;
    }

    agent->reply_end_last_char = last_ch;
    agent->reply_end_char = end_ch;

    return 0;
}

int AtSetReplyCharNum(ATAgentType agent, unsigned int num)
{  
    if (!agent) {
        return -1;
    }

    agent->reply_char_num = num;

    return 0;
}

int EntmSend(ATAgentType agent, const char *data, int len)
{
    char send_buf[128];
    if(len > 128){
        printf("send length %d more then max 128 Bytes.\n",len);
        return -1;
    }

    PrivMutexObtain(&agent->lock);
    memset(send_buf, 0, 128);

    agent->receive_mode = ENTM_MODE;

    memcpy(send_buf, data, len);
    // memcpy(send_buf + len, "!@", 2);

	PrivWrite(agent->fd, send_buf, len);
    PrivMutexAbandon(&agent->lock);
    printf("entm send %s length %d\n",send_buf, len);
    return 0;
}

int EntmRecv(ATAgentType agent, char *rev_buffer, int buffer_len, int timeout_s)
{
    struct timespec abstime;

    abstime.tv_sec = timeout_s;
    if(buffer_len > ENTM_RECV_MAX){
        printf("read length more then max length[%d] Bytes",ENTM_RECV_MAX);
      return -1;  
    }
    PrivMutexObtain(&agent->lock); 
    agent->receive_mode = ENTM_MODE;
    agent->read_len = buffer_len;
    PrivMutexAbandon(&agent->lock); 
    //PrivTaskDelay(1000);
    if (PrivSemaphoreObtainWait(&agent->entm_rx_notice, &abstime)) {
        printf("wait sem[%d] timeout\n",agent->entm_rx_notice);
        return -ERROR;
    }
    PrivMutexObtain(&agent->lock); 


    printf("EntmRecv once len %d.\n", agent->entm_recv_len);

    memcpy(rev_buffer, agent->entm_recv_buf, agent->entm_recv_len);

    memset(agent->entm_recv_buf, 0, ENTM_RECV_MAX);
    agent->entm_recv_len = 0;
    agent->read_len = 0;
    PrivMutexAbandon(&agent->lock); 

    return buffer_len;
}

static int GetCompleteATReply(ATAgentType agent)
{
    uint32_t read_len = 0;
    char ch = 0, last_ch = 0;
    bool is_full = false;

    PrivMutexObtain(&agent->lock); 

    memset(agent->maintain_buffer, 0x00, agent->maintain_max);
    agent->maintain_len = 0;

    memset(agent->entm_recv_buf, 0x00, 256);
    agent->entm_recv_len = 0;

    PrivMutexAbandon(&agent->lock); 

    while (1) 
    {
        PrivRead(agent->fd, &ch, 1);
#ifdef CONNECTION_FRAMEWORK_DEBUG
        if(ch != 0){
            printf(" %c (0x%x)\n", ch, ch);
        }
#endif

        PrivMutexObtain(&agent->lock);
        if (agent->receive_mode == ENTM_MODE)
        {
            if (agent->entm_recv_len < ENTM_RECV_MAX) 
            {
                agent->entm_recv_buf[agent->entm_recv_len] = ch;
                agent->entm_recv_len++;

                if(agent->entm_recv_len < agent->read_len){
                    PrivMutexAbandon(&agent->lock);
                    continue;
                }
                else 
                {
                    printf("ENTM_MODE recv %d Bytes done.\n",agent->entm_recv_len);
                    agent->receive_mode = DEFAULT_MODE;
                    PrivSemaphoreAbandon(&agent->entm_rx_notice);
                }

            } else {
                printf("entm_recv_buf is_full ...\n");
            }
        } 
        else if (agent->receive_mode == AT_MODE) 
        {
            if (read_len < agent->maintain_max)
            {
                if(ch != 0) ///< if the char is null then do not save it to the buff
                {
                    agent->maintain_buffer[read_len] = ch;
                    read_len++;
                    agent->maintain_len = read_len;
                }
                
            } else {
                printf("maintain_len is_full ...\n");
                is_full = true;
            }

            if (((ch == '\n') && (last_ch == '\r') && (agent->reply_lr_end)) || 
               ((ch == agent->reply_end_char) && (agent->reply_end_char) &&
                (last_ch == agent->reply_end_last_char) && (agent->reply_end_last_char)) ||
               ((read_len == agent->reply_char_num) && (agent->reply_char_num))){
                if (is_full) {
                    printf("read line failed. The line data length is out of buffer size(%d)!", agent->maintain_max);
                    memset(agent->maintain_buffer, 0x00, agent->maintain_max);
                    agent->maintain_len = 0;
                    PrivMutexAbandon(&agent->lock);
                    return -ERROR;
                }
               
                printf("GetCompleteATReply done\n");
                agent->receive_mode = DEFAULT_MODE;
                PrivMutexAbandon(&agent->lock);
                break;
            }
            last_ch = ch;
        }

        PrivMutexAbandon(&agent->lock);
    }

    return read_len;
}

ATAgentType GetATAgent(const char *agent_name)
{
    struct ATAgent* result = NULL;
    for (int i = 0; i < AT_AGENT_MAX; i++) {
        if (strcmp(at_agent_table[i].agent_name, agent_name) == 0) {
            result = &at_agent_table[i];
        }
    }

    return result;
}


int DeleteATAgent(ATAgentType agent)
{
    printf("delete agent->at_handler = %d\n",agent->at_handler);
    if(agent->at_handler > 0){
        PrivTaskDelete(agent->at_handler, 0);
    }
    
    if (agent->fd > 0) {
        printf("close agent fd = %d\n",agent->fd);
        PrivClose(agent->fd);
    }

#ifdef ADD_NUTTX_FETURES
    if (agent->lock.sem.semcount > 0) {
        printf("delete agent lock = %d\n",agent->lock.sem.semcount);
        PrivMutexDelete(&agent->lock);
    }
#else
    if (agent->lock) {
        printf("delete agent lock = %d\n",agent->lock);
        PrivMutexDelete(&agent->lock);
    }
#endif

    if (agent->entm_rx_notice) {
        printf("delete agent entm_rx_notice = %d\n",agent->entm_rx_notice);
        PrivSemaphoreDelete(&agent->entm_rx_notice);
    }

    if (agent->rsp_sem) {
        printf("delete agent rsp_sem = %d\n",agent->rsp_sem);
        PrivSemaphoreDelete(&agent->rsp_sem);
    }

    if (agent->maintain_buffer) {
        PrivFree(agent->maintain_buffer);
    }

    memset(agent, 0x00, sizeof(struct ATAgent));
    printf("delete ATagent\n");
}

static void *ATAgentReceiveProcess(void *param)
{
    ATAgentType agent = (ATAgentType)param;
    const struct at_urc *urc;

    while (1) {
        if (GetCompleteATReply(agent) > 0) 
        {
            PrivMutexObtain(&agent->lock); 
            if (agent->reply != NULL){
                ATReplyType reply = agent->reply;

                agent->maintain_buffer[agent->maintain_len] = '\0';
                if (agent->maintain_len <= reply->reply_max_len) {
                    memset(reply->reply_buffer, 0 , reply->reply_max_len);
                    memcpy(reply->reply_buffer, agent->maintain_buffer, agent->maintain_len);
                    reply->reply_len = agent->maintain_len;
                } else {
                    printf("out of memory (%d)!\n", reply->reply_max_len);
                }

                // agent->reply = NULL;
                PrivSemaphoreAbandon(&agent->rsp_sem);
            }
            PrivMutexAbandon(&agent->lock);
        }
    }
}

static int ATAgentInit(ATAgentType agent)
{
    int result = 0;

    agent->maintain_len = 0;
    agent->maintain_buffer = (char *)PrivMalloc(agent->maintain_max);

    if (agent->maintain_buffer == NULL) {
        printf("ATAgentInit malloc maintain_buffer error\n");
        goto __out;
    }

    memset(agent->maintain_buffer, 0, agent->maintain_max);

    result = PrivSemaphoreCreate(&agent->entm_rx_notice, 0, 0);
    if (result < 0) {
        printf("ATAgentInit create entm sem error\n");
        goto __out;
    }
    printf("create entm_rx_notice_sem %d\n ",agent->entm_rx_notice);
    result = PrivSemaphoreCreate(&agent->rsp_sem, 0, 0);
    if (result < 0) {
        printf("ATAgentInit create rsp sem error\n");
        goto __out;
    }
    printf("create rsp_sem %d\n ",agent->rsp_sem);
    if(PrivMutexCreate(&agent->lock, 0) < 0) {
        printf("AdapterFrameworkInit mutex create failed.\n");
        goto __out;
    }

    agent->receive_mode = DEFAULT_MODE;

#ifdef ADD_NUTTX_FETURES
    pthread_attr_t attr = PTHREAD_ATTR_INITIALIZER;
    attr.priority = 18;
    attr.stacksize = 4096;

#else
    pthread_attr_t attr;
    attr.schedparam.sched_priority = 25;
    attr.stacksize = 4096;
#endif

    PrivTaskCreate(&agent->at_handler, &attr, ATAgentReceiveProcess, agent);
    printf("create agent->at_handler = %d\n",agent->at_handler);
    return result;

__out:
    DeleteATAgent(agent);
    result = -1;

    return result;
}

int InitATAgent(const char *agent_name, int agent_fd, uint32 maintain_max)
{
    int i = 0;
    int result = 0;
    int open_result = 0;
    struct ATAgent *agent = NULL;

    if (GetATAgent(agent_name) != NULL) {
        return result;
    }
    
    while (i < AT_AGENT_MAX && at_agent_table[i].fd > 0) {
        i++;
    }

    if (i >= AT_AGENT_MAX) {
        printf("agent buffer(%d) is full.", AT_AGENT_MAX);
        result = -1;
        return result;
    }

    agent = &at_agent_table[i];

    agent->fd = agent_fd;

    strcpy(agent->agent_name, agent_name);

    agent->maintain_max = maintain_max;

    result = ATAgentInit(agent);
    if (result == 0) {
        PrivTaskStartup(&agent->at_handler);
    }

    return result;
}

ATReplyType CreateATReply(uint32 reply_max_len)
{
    ATReplyType reply = NULL;

    reply = (ATReplyType)PrivMalloc(sizeof(struct ATReply));
    if (reply == NULL) {
        printf("no more memory\n");
        return NULL;
    }
    memset(reply, 0, sizeof(struct ATReply));

    reply->reply_max_len = reply_max_len;

    reply->reply_buffer = (char *)PrivMalloc(reply_max_len);
    if (reply->reply_buffer == NULL) {
        printf("no more memory\n");
        PrivFree(reply);
        return NULL;
    }

    memset(reply->reply_buffer, 0, reply_max_len);

    return reply;
}

void DeleteATReply(ATReplyType reply)
{
    if (reply) {
        if (reply->reply_buffer) {
            PrivFree(reply->reply_buffer);
            reply->reply_buffer = NULL;
        }
    }

    if (reply) {
        PrivFree(reply);
        reply = NULL;
    }
}
