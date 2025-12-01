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
 * @file bc28.c
 * @brief Implement the connection nbiot adapter function, using BC28 device
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.09.15
 */

#include <adapter.h>
#include <at_agent.h>
#include "../adapter_nbiot.h"
#include <stdlib.h>

#define SOCKET_PROTOCOL_TCP  (6)
#define SOCKET_PROTOCOL_UDP  (17)

#define NET_TYPE_AF_INET   (0)
#define NET_TYPE_AF_INET6  (1)

#define SOCKET_INVALID_ID  (-1)
static int nbiot_lock;

static int BC28UartOpen(struct Adapter *adapter)
{
    if (NULL == adapter) {
        return -1;
    }

    /* Open device in read-write mode */
    adapter->fd = PrivOpen(ADAPTER_BC28_DRIVER,O_RDWR);
    if (adapter->fd < 0) {
        printf("BC28UartOpen get serial %s fd error\n", ADAPTER_BC28_DRIVER);
        return -1;
    }
    /* set serial config, serial_baud_rate = 9600 */

    struct SerialDataCfg cfg;
    memset(&cfg, 0 ,sizeof(struct SerialDataCfg));

    cfg.serial_baud_rate = BAUD_RATE_9600;
    cfg.serial_data_bits = DATA_BITS_8;
    cfg.serial_stop_bits = STOP_BITS_1;
    cfg.serial_parity_mode = PARITY_NONE;
    cfg.serial_bit_order = BIT_ORDER_LSB;
    cfg.serial_invert_mode = NRZ_NORMAL;
    cfg.serial_buffer_size = SERIAL_RB_BUFSZ;

    /*aiit board use ch438, so it needs more serial configuration*/
#ifdef ADAPTER_BC28_DRIVER_EXTUART
    cfg.ext_uart_no         = ADAPTER_BC28_DRIVER_EXT_PORT;
    cfg.port_configure      = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &cfg;

    PrivIoctl(adapter->fd, OPE_INT, &ioctl_cfg);
    PrivTaskDelay(1000);

    printf("NBIot uart config ready\n");
    return 0;
}

static void BC28PowerSet(void)
{
    int pin_fd;
    pin_fd = PrivOpen(ADAPTER_BC28_PIN_DRIVER, O_RDWR);

    struct PinParam pin_param;
    pin_param.cmd = GPIO_CONFIG_MODE;
    pin_param.mode = GPIO_CFG_OUTPUT; 
    pin_param.pin = ADAPTER_BC28_RESETPIN;

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = PIN_TYPE;
    ioctl_cfg.args = &pin_param;
    PrivIoctl(pin_fd, OPE_CFG, &ioctl_cfg);

    struct PinStat pin_stat;
    pin_stat.pin = ADAPTER_BC28_RESETPIN;
    pin_stat.val = GPIO_HIGH;
    PrivWrite(pin_fd, &pin_stat, 1);

    PrivTaskDelay(200);//at least 200ms

    pin_stat.val = GPIO_LOW;
    PrivWrite(pin_fd, &pin_stat, 1);

    PrivClose(pin_fd);
}

int NBIoTStatusCheck(struct Adapter *adapter )
{
    int32 result = 0;

    if (!adapter ){
        return -1;
    }

    char at_cmd[64] = {0};

    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B); /* set receive end flag as 'OK'*/
    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+NCONFIG=AUTOCONNECT,TRUE", 27);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    ATOrderSend(adapter->agent, 0, NULL, at_cmd);

    PrivTaskDelay(10000);

    // memset(at_cmd, 0 ,64);
    // memcpy(at_cmd, "AT+NRB", 6);
    // strcat(at_cmd, "\n");
    // printf("cmd : %s\n", at_cmd);
    // result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    // if(result < 0) {
    //     printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
    //     result = -1;
    //     goto out;
    // }
    // BC28PowerSet(); /* reset bc28 module by set reset pin */
    // PrivTaskDelay(6000);

    // memset(at_cmd, 0 ,64);
    // memcpy(at_cmd, "AT+NBAND=5", 10);
    // strcat(at_cmd, "\n");
    // printf("cmd : %s\n", at_cmd);
    // result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    // if(result < 0) {
    //     printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
    //     result = -1;
    //     goto out;
    // }
    

    // memset(at_cmd, 0 ,64);
    // memcpy(at_cmd, "AT+CFUN=1", 10);
    // strcat(at_cmd, "\n");
    // printf("cmd : %s\n", at_cmd);
    // result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    // if(result < 0) {
    //     printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
    //     result = -1;
    //     goto out;
    // }

    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B); /* set receive end flag as 'OK'*/
    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+CFUN?", 8);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
        goto out;
    }

    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+CIMI", 7);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
        goto out;
    }

    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+CEREG?", 9);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, ",1");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
        goto out;
    }

    // memset(at_cmd, 0 ,64);
    // memcpy(at_cmd, "AT+CGATT=1", 10);
    // strcat(at_cmd, "\n");
    // printf("cmd : %s\n", at_cmd);
    // result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    // if(result < 0) {
    //     printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
    //     result = -1;
    //     goto out;
    // }

    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+CGATT?", 9);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
        goto out;
    }

    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+CSQ", 6);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
        goto out;
    }
 
    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+CGPADDR", 10);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
    }
    
    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+QREGSWT=2", 12);
    strcat(at_cmd, "\n");
    printf("cmd : %s\n", at_cmd);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
        goto out;
    }

out:
    return result;
}

/**
 * @description: NBIoT device create a socket connection
 * @param adapter - NBIoT adapter AT
 * @param socket_fd - socket file description
 * @param type - socket type
 * @param af_type - IPv4 or IPv6
 * @return success: EOK, failure: -ERROR
 */
int NBIoTSocketCreate(struct Adapter *adapter, struct Socket *socket )
{
    int32 result = 0;

    if (!adapter || !socket){
        return -1;
    }

    if ( socket->af_type == NET_TYPE_AF_INET6 ) {
        printf("IPv6 not surport !\n");
        result = -1;
        goto out;
    }

    char *str_af_type = "AF_INET";
    char *str_type;
    char str_fd[3] = {1};
    char *str_protocol ;
    char at_cmd[64] = {0};
    char listen_port[] = {0};

    if (socket->socket_id >= 0 && socket->socket_id < 7) {
        itoa(socket->socket_id, str_fd, 10);
        adapter->socket.socket_id = socket->socket_id;
    } else {
        printf("surport max 0-6, socket_id = [%d] is error!\n",socket->socket_id);
        result = -1;
        goto out;
    }

    if( socket->listen_port >= 0 && socket->listen_port <= 65535){
        itoa(socket->listen_port, listen_port, 10);
    } 
    
    adapter->socket.af_type = NET_TYPE_AF_INET;

    if (socket->type == SOCKET_TYPE_STREAM) {   //tcp = AT+NSOCR=STREAM,6,0,1,AF_INET
        adapter->socket.protocal = SOCKET_PROTOCOL_TCP;
        adapter->socket.type = SOCKET_TYPE_STREAM;
        str_type = "STREAM";
        str_protocol = "6";

    } else if ( socket->type == SOCKET_TYPE_DGRAM ){  //udp
        adapter->socket.type = SOCKET_TYPE_DGRAM;
        adapter->socket.protocal = SOCKET_PROTOCOL_UDP;
        str_type = "DGRAM";
        str_protocol = "17";

    } else {
        printf("error socket type \n");
        result = -1;
        goto out;
    }
    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+NSOCR=", 9);
    strcat(at_cmd, str_type);
    strcat(at_cmd, ",");
    strcat(at_cmd, str_protocol);
    strcat(at_cmd, ",");
    strcat(at_cmd, listen_port);
    strcat(at_cmd, ",");
    strcat(at_cmd, str_fd);
    strcat(at_cmd, ",");
    strcat(at_cmd, str_af_type);
    strcat(at_cmd, "\n");

    printf("cmd : %s\n", at_cmd);

    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B); /* set receive end flag as 'OK'*/

    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
    }

out:
    return result;
}

static int BC28ParseData(void *dst, ATReplyType reply)
{
    int nbdata_len = 0;
    int check_cnt = 0;
    int left = 0;
    int right = 0;
    int tag = 0;
    char *nbdata = NULL;
    char *result = NULL;

    if (NULL == reply) {
        printf("at create failed ! \n");
        return -1;
    }

    result = GetReplyText(reply);
    if (!result) {
        printf("%s %n get reply failed.\n",__func__,__LINE__);
        return -1;
    }

    /*the data style of BC28: eg.'<socket id>,<ip addr>,<port>,<data length>,<data>,<data length>,' ,its assic code*/

    /*step1: get the <data> section between the fourth and fifth ','*/
    for(int i = 0; i < reply->reply_len; i++) 
    {
        if(',' == *(result + i)) 
        {
            check_cnt++;
            if(4 == check_cnt) /*the fourth ','*/
            {
                left = i;
            }

            if(5 == check_cnt) /*the fifth ',' */
            {
                right = i;
                break;
            }
        }   
    }
    if(left > 0 && right > 0)
    {
        /*step2: transform assic code as hexadecimal*/
        for(int j = (left + 1);j <= (right - 1);j++) 
        {
            if(*(result + j) >= 0x30 && *(result + j) <= 0x39){
                *(result + j) = (*(result + j) - 0x30); /* transform [0-9]*/
            }
            else if(*(result + j) >= 0x41 && *(result + j) <= 0x46)
            {
                *(result + j) = (*(result + j) - 0x37); /* transform [A-F],NOTE!!! A-F can not be regarded as a char,only can be reagrded as a number*/
            }
            
            printf("0x%x ",*(result + j));
        }

        printf("\n");
        nbdata_len = (right - left - 1) / 2; /*caculate the data length */
        nbdata = PrivMalloc(nbdata_len);
        if (!nbdata) {
            printf("%s %n malloc failed.\n",__func__,__LINE__);
            return -1;
        }

        tag = left;
        printf("left = %d right = %d nbdata_len = %d\n",left ,right ,nbdata_len);
        
        /*step3: transform hexadecimal as actual data */
        for(int k = 0; k < nbdata_len; k++ )
        {
            *(nbdata + k) =  (*(result + tag + 1) * 16 + *(result + tag + 2));
            tag = tag + 2; /* transform with two data*/
            printf("0x%x ",*(nbdata + k));
        }
        printf("\n");

        memcpy(dst, nbdata, nbdata_len);

        PrivFree(nbdata);

        return nbdata_len;
    }
    else
    {
        return 0;
    }
    
}

/**
 * @description: NBIoT device close a socket connection
 * @param adapter - NBIoT adapter AT
 * @param socket_fd - socket file description
 * @return success: EOK, failure: -ERROR
 */
int NBIoTSocketDelete(struct Adapter *adapter )
{

    if (!adapter) {
        return -1;
    }

    if (adapter->socket.socket_id >= 7) {
        printf("socket fd error \n");
        return -1;
    }

    char str_fd[2] = {0};
    char at_cmd[32] = {0};
    printf("NBIOT close socket id = %d\n",adapter->socket.socket_id);
    itoa(adapter->socket.socket_id, str_fd, 10);

    memset(at_cmd, 0 ,32);
    memcpy(at_cmd, "AT+NSOCL=", 9);
    strcat(at_cmd, str_fd);
    strcat(at_cmd, "\n");

    printf("cmd : %s\n", at_cmd);
    AtSetReplyCharNum(adapter->agent, 1);
    ATOrderSend(adapter->agent, REPLY_TIME_OUT, NULL, at_cmd);
    PrivTaskDelay(300);

    adapter->socket.socket_id = SOCKET_INVALID_ID;

    return 0;
}

static int BC28Open(struct Adapter *adapter)
{
    int ret = 0;
    struct Socket create_socket;

    if (NULL == adapter) {
        return -1;
    }

    if(PrivMutexCreate(&nbiot_lock, 0) < 0) {
        printf("nbiot_lock mutex create failed.\n");
        return -1;
    }

    /*step1: open BC8 serial port*/
    ret = BC28UartOpen(adapter);
    if (ret < 0) {
        printf("bc18 setup failed.\n");
        PrivMutexDelete(&nbiot_lock);
        return -1;
    }

     /*step2: init AT agent*/
    if (!adapter->agent) {
        char *agent_name = "niot_device";
        if (EOK != InitATAgent(agent_name, adapter->fd, 512)) {
            PrivClose(adapter->fd);
            PrivMutexDelete(&nbiot_lock);
            printf("at agent init failed !\n");
            return -1;
        }
        ATAgentType at_agent = GetATAgent(agent_name);
        adapter->agent = at_agent;
    }

    // BC28PowerSet(); /* reset bc28 module by set reset pin */
    PrivTaskDelay(6000);

    ret = NBIoTStatusCheck(adapter); /* ask module status*/
    if(ret < 0){
        DeleteATAgent(adapter->agent);
        adapter->agent = NULL;
        PrivMutexDelete(&nbiot_lock);
        printf("NBIot status check failed.\n");
        return -1;
    }

    create_socket.type = SOCKET_TYPE_STREAM;
    create_socket.listen_port = 0;
    create_socket.socket_id = 1;
    create_socket.af_type = NET_TYPE_AF_INET;

    /*step3: create a tcp socket default */
    ret = NBIoTSocketCreate(adapter, &create_socket);
    if(ret < 0) {
        DeleteATAgent(adapter->agent);
        adapter->agent = NULL;
        PrivMutexDelete(&nbiot_lock);
        printf("NBIot create tcp socket failed.\n");
        return -1;
    }

    printf("NBiot BC28 open successful\n");
    return 0;
}

static int BC28Close(struct Adapter *adapter)
{
   
    if(&adapter->socket){
        NBIoTSocketDelete(adapter);
    }

    if(adapter->agent)
    {
        DeleteATAgent(adapter->agent);
        adapter->agent = NULL;
    }

    PrivMutexDelete(&nbiot_lock);
    return 0;
}

static int BC28Ioctl(struct Adapter *adapter, int cmd, void *args)
{
    int ret = 0;
    PrivMutexObtain(&nbiot_lock);
    switch (cmd)
    {
        case CONFIG_NBIOT_RESET: /* reset nbiot */
            BC28PowerSet();
            break;
        case CONFIG_NBIOT_CREATE_SOCKET: /* create tcp/UDP socket */
            if(!args){
                return -1;
            }
            struct Socket *create_socket = ( struct Socket *) args;
            ret = NBIoTSocketCreate(adapter, create_socket);
            break;
        case CONFIG_NBIOT_DELETE_SOCKET: /* close socket */
            ret = NBIoTSocketDelete(adapter);
            break;
        default:
            ret = -1;
            break;
    }
    PrivMutexAbandon(&nbiot_lock);
    return ret;
}

static int BC28Connect(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type)
{
    int result = 0;

    if (adapter->socket.socket_id > 6) {
        printf("socket fd error \n");
        result = -1;
        goto __exit;
    }

    if ( ip_type != SOCKET_TYPE_STREAM) {
        printf("socket type error \n");
        result = -1;
        goto __exit;
    }

    char at_cmd[64] = {0};
    char str_fd[2] = {0};

    PrivMutexObtain(&nbiot_lock);
    itoa(adapter->socket.socket_id, str_fd, 10);

    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+NSOCO=", 9);
    strcat(at_cmd, str_fd);
    strcat(at_cmd, ",");
    strcat(at_cmd, ip);
    strcat(at_cmd, ",");
    strcat(at_cmd, port);
    strcat(at_cmd, "\n");

    printf("cmd : %s\n", at_cmd);
    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
    }
    PrivMutexAbandon(&nbiot_lock);
__exit:
    return result;
}

static int BC28Send(struct Adapter *adapter, const void *buf, size_t len)
{
    uint32_t result = 0;
    char at_cmd[64] = {0};
    char str_fd[2] = {0};
    char assic_str[2] = {0};
    int assic_val = 0;
    int length = 0;

    PrivMutexObtain(&nbiot_lock);

    char *nbdata = PrivMalloc(2 * len);
    memset(nbdata, 0 ,2 * len);

    for(int i = 0; i < len; i++)
    {
        assic_val = 0;
        for( ; assic_val <= 0x7f; assic_val++) 
        {
            if(*(char*)(buf + i) == assic_val) 
            {
                break;
            }
        }
        itoa(assic_val, assic_str, 16);
        memcpy(nbdata + length, assic_str, 2);
        length = length + 2;
    }
    if (adapter->socket.type == SOCKET_TYPE_STREAM ) {

        char size[2] = {0};

        itoa(adapter->socket.socket_id, str_fd, 10);
        size[0] = len + '0';

        memset(at_cmd, 0 ,64);
        memcpy(at_cmd, "AT+NSOSD=", 9);
        strcat(at_cmd, str_fd);
        strcat(at_cmd, ",");
        strcat(at_cmd, size);
        strcat(at_cmd, ",");
        strncat(at_cmd, nbdata,length);
        strcat(at_cmd, "\n");

    } else if(adapter->socket.type == SOCKET_TYPE_DGRAM ) {

        char listen_port[] = {0};

        itoa(adapter->socket.socket_id, str_fd, 10);

        itoa(adapter->socket.listen_port, listen_port, 10);

        memset(at_cmd, 0 ,64);
        memcpy(at_cmd, "AT+NSOST=", 9);
        strcat(at_cmd, str_fd);
        strcat(at_cmd, ",");
        strcat(at_cmd, adapter->socket.dst_ip_addr);
        strcat(at_cmd, ",");
        strcat(at_cmd, listen_port);
        strcat(at_cmd, ",");
        strncat(at_cmd, nbdata,length);
        strcat(at_cmd, "\n");

    }
    PrivFree(nbdata);
    printf("cmd : %s\n", at_cmd);
    AtSetReplyEndChar(adapter->agent, 0x4F, 0x4B);
    result = AtCmdConfigAndCheck(adapter->agent, at_cmd, "OK");
    if(result < 0) {
        printf("%s %d cmd[%s] config failed!\n",__func__,__LINE__,at_cmd);
        result = -1;
    }
    PrivMutexAbandon(&nbiot_lock);
    return result;
}


static int BC28Recv(struct Adapter *adapter, void *buf, size_t len)
{
    char at_cmd[64] = {0};
    char str_fd[2] = {0};
    char size[2] = {0};
    int ret = 0;
    char *result = NULL;
    
    PrivMutexObtain(&nbiot_lock);

    ATReplyType reply = CreateATReply(512);
    if (NULL == reply) {
        printf("at create failed ! \n");
        PrivMutexAbandon(&nbiot_lock);
        return -1;
    }
    
    itoa(adapter->socket.socket_id, str_fd, 10);
    itoa(len, size, 10);

    memset(at_cmd, 0 ,64);
    memcpy(at_cmd, "AT+NSORF=", 9);
    strcat(at_cmd, str_fd);
    strcat(at_cmd, ",");
    strcat(at_cmd, size);
    strcat(at_cmd, "\n");

    printf("cmd : %s\n", at_cmd);
    ret = ATOrderSend(adapter->agent, REPLY_TIME_OUT, reply, at_cmd);
    if(ret < 0)
    {
        printf("NBiot receive timeout\n");
        DeleteATReply(reply);
        return -1;
    }

    if(reply)
    {
        ret = BC28ParseData(buf, reply);
        if (ret < 0)
        {
            PrivMutexAbandon(&nbiot_lock);
            if (reply)
            {
                DeleteATReply(reply);
            }
            return ret;
        }
        DeleteATReply(reply);
    }
    
    PrivMutexAbandon(&nbiot_lock);
    return ret;
}

static int BC28Disconnect(struct Adapter *adapter)
{
    if (!adapter) {
        return -1;
    }

    return NBIoTSocketDelete(adapter);
}

static const struct IpProtocolDone BC28_done = 
{
    .open = BC28Open,
    .close = BC28Close,
    .ioctl = BC28Ioctl,
    .setup = NULL,
    .setdown = NULL,
    .setaddr = NULL,
    .setdns = NULL,
    .setdhcp = NULL,
    .ping = NULL,
    .netstat = NULL,
    .connect = BC28Connect,
    .send = BC28Send,
    .recv = BC28Recv,
    .disconnect = BC28Disconnect,
};

AdapterProductInfoType BC28Attach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = PrivMalloc(sizeof(struct AdapterProductInfo));
    if (!product_info) {
        printf("BC28Attach malloc product_info error\n");
        return NULL;
    }
    memset(product_info, 0, sizeof(struct AdapterProductInfo));

    strncpy(product_info->model_name, ADAPTER_NBIOT_BC28,sizeof(product_info->model_name));
    product_info->model_done = (void *)&BC28_done;

    return product_info;
}



