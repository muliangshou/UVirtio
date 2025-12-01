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
 * @file adapter_lora.c
 * @brief Implement the connection lora adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.10.20
 */

#include <adapter.h>

#ifdef ADAPTER_SX1278
extern AdapterProductInfoType Sx1278Attach(struct Adapter *adapter);
#endif

#ifdef ADAPTER_E220
extern AdapterProductInfoType E220Attach(struct Adapter *adapter);
#endif

#define ADAPTER_LORA_CLIENT_NUM 255
#define ADAPTER_LORA_DATA_LENGTH 256
#define ADAPTER_LORA_RECV_DATA_LENGTH ADAPTER_LORA_DATA_LENGTH + 16

#define ADAPTER_LORA_DATA_HEAD            0x3C
#define ADAPTER_LORA_NET_PANID            0x0102
#define ADAPTER_LORA_DATA_TYPE_JOIN       0x0A
#define ADAPTER_LORA_DATA_TYPE_QUIT       0x0B
#define ADAPTER_LORA_DATA_TYPE_JOIN_REPLY 0x0C
#define ADAPTER_LORA_DATA_TYPE_QUIT_REPLY 0x0D
#define ADAPTER_LORA_DATA_TYPE_USERDATA   0x0E
#define ADAPTER_LORA_DATA_TYPE_CMD        0x0F

//need to change status if the lora client wants to quit the net when timeout or a certain event
//eg.can also use sem to trigger quit function
static int g_adapter_lora_quit_flag = 0;

enum ClientState
{
    CLIENT_DISCONNECT = 0,
    CLIENT_CONNECT,
    CLIENT_DEFAULT,
};

enum DataType
{
    LORA_CLIENT_ACCESS = 0,
    LORA_GATEWAY_REPLY,
    LORA_USER_DATA,
};

enum LoraGatewayState {
    LORA_STATE_IDLE = 0,
    LORA_JOIN_NET,
    LORA_QUIT_NET,
    LORA_RECV_DATA,
};

static uint8 LoraGatewayState = LORA_STATE_IDLE;

struct LoraGatewayParam 
{
    uint8 gateway_id;
    uint16 panid;
    uint8 client_id[ADAPTER_LORA_CLIENT_NUM];
    int client_num;
    int receive_data_sem;
};

struct LoraClientParam 
{
    uint8 client_id;
    uint16 panid;
    enum ClientState client_state;

    uint8 gateway_id;
};

/*LoraDataFormat:
**flame_head : 0x3C
**length     : sizeof(struct LoraDataFormat)
**panid      : 0x0102
**client_id  : 0x01、0x02、0x03...
**gateway_id : 0x11、0x12、0x13...
**data_type  : 0x0A(join net, client->gateway)、0x0B(join net reply, gateway->client)、
               0x0C(user data, client->gateway)、0x0D(user data cmd, gateway->client)
**data       : user data
**crc16      : CRC 16 check data
*/
struct LoraDataFormat
{
    uint8 flame_head;
    uint8 reserved[3];
    uint32 length;
    uint16 panid;
    uint8 client_id;
    uint8 gateway_id;

    uint16 data_type;
    uint8 data[ADAPTER_LORA_DATA_LENGTH];

    uint16 crc16;
};

/*******************LORA MESH FUNCTION***********************/

/**
 * @description: create CRC16 data
 * @param data input data buffer
 * @param length input data buffer minus check code
 */
static uint16 LoraCrc16(uint8 *data, uint16 length)
{
    int j;
    uint16 crc_data = 0xFFFF;
    
    while (length--) {
        crc_data ^= *data++;
        for( j = 0 ; j < 8 ; j ++) {
            if(crc_data & 0x01) /* LSB(b0)=1 */
                crc_data = crc_data >> 1 ^ 0xA001;
            else
                crc_data = crc_data >> 1;
        }
    }
    
    return crc_data;
}

/**
 * @description: CRC16 check
 * @param data input data buffer
 * @param length input data buffer minus check code
 */
static int LoraCrc16Check(uint8 *data, uint16 length)
{
    uint16 crc_data;
    uint16 input_data = (((uint16)data[length - 1] << 8) & 0xFF00) | ((uint16)data[length - 2] & 0x00FF);

    crc_data = LoraCrc16(data, length - 2);

    if (crc_data == input_data)
        return 0;
    else
        return -1;
}

/**
 * @description: Lora receive data check
 * @param data receive data buffer
 * @param length receive data length
 * @param recv_data LoraDataFormat data
 */
static int LoraReceiveDataCheck(uint8 *data, uint16 length, struct LoraDataFormat *recv_data)
{
    int i;
    uint32 recv_data_length = 0;
    for ( i = 0; i < length; i ++) {
        if (ADAPTER_LORA_DATA_HEAD == data[i]) {
            recv_data_length = (data[i + 4] & 0xFF) | ((data[i + 5] & 0xFF) << 8) | ((data[i + 6] & 0xFF) << 16) | ((data[i + 7] & 0xFF) << 24);
            if (sizeof(struct LoraDataFormat) == recv_data_length) {
                memcpy(recv_data, (uint8 *)(data + i), sizeof(struct LoraDataFormat));
                return 0;
            }
        }
    }

    return -1;
}

/**
 * @description: Lora Gateway reply connect request to Client
 * @param adapter Lora adapter pointer
 * @param gateway_recv_data Lora Client connect data
 */
static int LoraGatewayReply(struct Adapter *adapter, struct LoraDataFormat *gateway_recv_data)
{
    int i;
    int client_join_flag = 0;
    struct LoraGatewayParam *gateway = (struct LoraGatewayParam *)adapter->adapter_param;
    struct LoraDataFormat gateway_reply_data;

    memset(&gateway_reply_data, 0, sizeof(struct LoraDataFormat));

    if (ADAPTER_LORA_DATA_TYPE_JOIN == gateway_recv_data->data_type) {
        
        for (i = 0; i < gateway->client_num; i ++) {
            if (gateway_recv_data->client_id == gateway->client_id[i]) {
                printf("Lora client_%d 0x%x has already join net 0x%x\n", i, gateway_recv_data->client_id, gateway->gateway_id);
                client_join_flag = 1;
                break;
            }
        }
        
        if (!client_join_flag) {
            if (gateway->client_num > 6) {
                printf("Lora gateway only support 6(max) client\n");
                gateway->client_num = 0;
            }
            gateway->client_id[gateway->client_num] = gateway_recv_data->client_id;
            gateway->client_num ++;
        }

        gateway_reply_data.flame_head = ADAPTER_LORA_DATA_HEAD;
        gateway_reply_data.length = sizeof(struct LoraDataFormat);
        gateway_reply_data.panid = gateway->panid;
        gateway_reply_data.data_type = ADAPTER_LORA_DATA_TYPE_JOIN_REPLY;
        gateway_reply_data.client_id = gateway_recv_data->client_id;
        gateway_reply_data.gateway_id = gateway->gateway_id;
        gateway_reply_data.crc16 = LoraCrc16((uint8 *)&gateway_reply_data, sizeof(struct LoraDataFormat) - 2);
        
        if (AdapterDeviceSend(adapter, (uint8 *)&gateway_reply_data, gateway_reply_data.length) < 0) {
            return -1;
        }

        printf("Lora gateway 0x%x accept client 0x%x <join request>\n", gateway->gateway_id, gateway_recv_data->client_id);
    } else if (ADAPTER_LORA_DATA_TYPE_QUIT == gateway_recv_data->data_type) {
        for (i = 0; i < gateway->client_num; i ++) {
            if (gateway->client_id[i] == gateway_recv_data->client_id) {
                gateway->client_id[i] = 0;
                gateway->client_num --;
                break;
            }
        }

        gateway_reply_data.flame_head = ADAPTER_LORA_DATA_HEAD;
        gateway_reply_data.length = sizeof(struct LoraDataFormat);
        gateway_reply_data.panid = gateway->panid;
        gateway_reply_data.data_type = ADAPTER_LORA_DATA_TYPE_QUIT_REPLY;
        gateway_reply_data.client_id = gateway_recv_data->client_id;
        gateway_reply_data.gateway_id = gateway->gateway_id;
        gateway_reply_data.crc16 = LoraCrc16((uint8 *)&gateway_reply_data, sizeof(struct LoraDataFormat) - 2);
        
        if (AdapterDeviceSend(adapter, (uint8 *)&gateway_reply_data, gateway_reply_data.length) < 0) {
            return -1;
        }

        printf("Lora gateway 0x%x accept client 0x%x <quit request>\n", gateway->gateway_id, gateway_recv_data->client_id);
    }

    return 0;
}

/**
 * @description: Lora Gateway send cmd to Client
 * @param adapter Lora adapter pointer
 * @param client_id Lora Client id
 * @param cmd Lora cmd
 */
static int LoraGatewaySendCmd(struct Adapter *adapter, unsigned char client_id, int cmd)
{
    struct LoraGatewayParam *gateway = (struct LoraGatewayParam *)adapter->adapter_param;
    struct LoraDataFormat gateway_cmd_data;

    memset(&gateway_cmd_data, 0, sizeof(struct LoraDataFormat));

    gateway_cmd_data.flame_head = ADAPTER_LORA_DATA_HEAD;
    gateway_cmd_data.length = sizeof(struct LoraDataFormat);
    gateway_cmd_data.panid = gateway->panid;
    gateway_cmd_data.data_type = cmd;
    gateway_cmd_data.client_id = client_id;
    gateway_cmd_data.gateway_id = gateway->gateway_id;
    gateway_cmd_data.crc16 = LoraCrc16((uint8 *)&gateway_cmd_data, sizeof(struct LoraDataFormat) - 2);
    
    if (AdapterDeviceSend(adapter, (uint8 *)&gateway_cmd_data, gateway_cmd_data.length) < 0) {
        return -1;
    }

    return 0;
}

/**
 * @description: Lora Gateway handle the user data from the client
 * @param adapter Lora adapter pointer
 * @param gateway_recv_data Lora Client user data
 */
static int LoraGatewayHandleData(struct Adapter *adapter, struct LoraDataFormat *gateway_recv_data)
{    
    /*User needs to handle client data depends on the requirement*/
    printf("Lora Gateway receive Client %d data:\n", gateway_recv_data->client_id);
    printf("%s\n", gateway_recv_data->data);
    return 0;
}

/**
 * @description: Lora Client update status after join the net
 * @param adapter Lora adapter pointer
 * @param client_recv_data Lora Client recv data from Lora Gateway
 */
static int LoraClientUpdate(struct Adapter *adapter, struct LoraDataFormat *client_recv_data)
{
    struct LoraClientParam *client = (struct LoraClientParam *)adapter->adapter_param;

    if (ADAPTER_LORA_DATA_TYPE_JOIN_REPLY == client_recv_data->data_type) {
        client->gateway_id = client_recv_data->gateway_id;
        client->panid = client_recv_data->panid;
        client->client_state = CLIENT_CONNECT;
        printf("LoraClientUpdate client join panid 0x%x successfully\n", client->panid);
    } else if (ADAPTER_LORA_DATA_TYPE_QUIT_REPLY == client_recv_data->data_type) {
        client->gateway_id = 0;
        client->panid = 0;
        client->client_state = CLIENT_DISCONNECT;
        printf("LoraClientUpdate client quit panid 0x%x successfully\n", client->panid);
    }

    return 0;
}

/**
 * @description: Lora Client send the user data to the gateway
 * @param adapter Lora adapter pointer
 * @param send_buf Lora Client send user data buf
 * @param length user data length (max size is ADAPTER_LORA_DATA_LENGTH)
 */
static int LoraClientSendData(struct Adapter *adapter, void *send_buf, int length)
{
    struct LoraClientParam *client = (struct LoraClientParam *)adapter->adapter_param;

    if (client->client_state != CLIENT_CONNECT) {
        printf("Lora client %d do not connect to Lora gateway\n", client->client_id);
        return 0;
    }

    if (length > ADAPTER_LORA_DATA_LENGTH) {
        printf("Lora client %d send data %d larger than max %d \n", client->client_id, length, ADAPTER_LORA_DATA_LENGTH);
        return 0;
    }

    struct LoraDataFormat client_user_data;

    memset(&client_user_data, 0, sizeof(struct LoraDataFormat));

    client_user_data.flame_head = ADAPTER_LORA_DATA_HEAD;
    client_user_data.length = sizeof(struct LoraDataFormat);
    client_user_data.panid = client->panid;
    client_user_data.data_type = ADAPTER_LORA_DATA_TYPE_USERDATA;
    client_user_data.client_id = client->client_id;
    client_user_data.gateway_id = client->gateway_id;

    memcpy(client_user_data.data, (uint8 *)send_buf, length);

    client_user_data.crc16 = LoraCrc16((uint8 *)&client_user_data, sizeof(struct LoraDataFormat) - 2);
    
    if (AdapterDeviceSend(adapter, (uint8 *)&client_user_data, client_user_data.length) < 0) {
        return -1;
    }

    return 0;
}

/**
 * @description: Lora Gateway receive data analyzing
 * @param adapter Lora adapter pointer
 * @param gateway_recv_data Lora gateway receive data pointer
 */
static int LoraGateWayDataAnalyze(struct Adapter *adapter, struct LoraDataFormat *gateway_recv_data)
{
    int ret = 0;
    printf("%s:gateway_recv_data\n",__func__);
    printf("head 0x%x length %d panid 0x%x data_type 0x%x client_id 0x%x gateway_id 0x%x crc 0x%x\n",
        gateway_recv_data->flame_head, gateway_recv_data->length, gateway_recv_data->panid, gateway_recv_data->data_type,
        gateway_recv_data->client_id, gateway_recv_data->gateway_id, gateway_recv_data->crc16);

    if (LoraCrc16Check((uint8 *)gateway_recv_data, sizeof(struct LoraDataFormat)) < 0) {
        printf("LoraGateWayDataAnalyze CRC check error\n");
        return -1;
    }

    if ((ADAPTER_LORA_DATA_HEAD == gateway_recv_data->flame_head) && 
        (ADAPTER_LORA_NET_PANID == gateway_recv_data->panid)) {
        switch (gateway_recv_data->data_type)
        {
        case ADAPTER_LORA_DATA_TYPE_JOIN : 
        case ADAPTER_LORA_DATA_TYPE_QUIT :
            ret = LoraGatewayReply(adapter, gateway_recv_data);
            break;
        case ADAPTER_LORA_DATA_TYPE_USERDATA :
            ret = LoraGatewayHandleData(adapter, gateway_recv_data);
            break;
        default:
            break;
        }
    }

    return ret;
}

/**
 * @description: Lora Client receive data analyzing
 * @param adapter Lora adapter pointer
 * @param send_buf Lora Client send user data buf
 * @param length user data length (max size is ADAPTER_LORA_DATA_LENGTH)
 */
static int LoraClientDataAnalyze(struct Adapter *adapter, void *send_buf, int length)
{
    int ret = 0;
    uint8 lora_recv_data[ADAPTER_LORA_RECV_DATA_LENGTH] = {0};
    
    struct LoraDataFormat *client_recv_data = PrivMalloc(sizeof(struct LoraDataFormat));

    memset(client_recv_data, 0, sizeof(struct LoraDataFormat));
    
    ret = AdapterDeviceRecv(adapter, lora_recv_data, ADAPTER_LORA_RECV_DATA_LENGTH);
    if (ret <= 0) {
        printf("LoraClientDataAnalyze recv error.Just return\n");
        PrivFree(client_recv_data);
        return -1;
    }

    LoraReceiveDataCheck(lora_recv_data, ADAPTER_LORA_RECV_DATA_LENGTH, client_recv_data);

    printf("%s:client_recv_data\n", __func__);
    printf("head 0x%x length %d panid 0x%x data_type 0x%x client_id 0x%x gateway_id 0x%x crc 0x%x\n",
        client_recv_data->flame_head, client_recv_data->length, client_recv_data->panid, client_recv_data->data_type,
        client_recv_data->client_id, client_recv_data->gateway_id, client_recv_data->crc16);

    if ((ADAPTER_LORA_DATA_HEAD == client_recv_data->flame_head) && 
        (ADAPTER_LORA_NET_PANID == client_recv_data->panid)) {
        if (LoraCrc16Check((uint8 *)client_recv_data, sizeof(struct LoraDataFormat)) < 0) {
            printf("LoraClientDataAnalyze CRC check error\n");
            PrivFree(client_recv_data);
            return -1;
        }

        //only handle this client_id information from gateway
        if (client_recv_data->client_id == adapter->net_role_id) {
            switch (client_recv_data->data_type)
            {
            case ADAPTER_LORA_DATA_TYPE_JOIN_REPLY : 
            case ADAPTER_LORA_DATA_TYPE_QUIT_REPLY :
                ret = LoraClientUpdate(adapter, client_recv_data);
                break;
            case ADAPTER_LORA_DATA_TYPE_CMD :
                if (send_buf) {
                    ret = LoraClientSendData(adapter, send_buf, length);
                }
                break;
            default:
                break;
            }
        }
    }

    PrivFree(client_recv_data);
    return ret;
}

/**
 * @description: Lora Client join net function
 * @param adapter Lora adapter pointer
 * @param panid Lora net panid
 */
static int LoraClientJoinNet(struct Adapter *adapter, unsigned short panid)
{
    struct LoraDataFormat client_join_data;
    struct AdapterData priv_lora_net;

    memset(&client_join_data, 0, sizeof(struct LoraDataFormat));

    client_join_data.flame_head = ADAPTER_LORA_DATA_HEAD;
    client_join_data.length = sizeof(struct LoraDataFormat);
    client_join_data.panid = panid;
    client_join_data.data_type = ADAPTER_LORA_DATA_TYPE_JOIN;
    client_join_data.client_id = adapter->net_role_id;
    client_join_data.crc16 = LoraCrc16((uint8 *)&client_join_data, sizeof(struct LoraDataFormat) - 2);

    printf("%s:client_join_data\n", __func__);
    printf("head 0x%x length %d panid 0x%x data_type 0x%x client_id 0x%x gateway_id 0x%x crc 0x%x\n",
        client_join_data.flame_head, client_join_data.length, client_join_data.panid, client_join_data.data_type,
        client_join_data.client_id, client_join_data.gateway_id, client_join_data.crc16);
    
    priv_lora_net.len = sizeof(struct LoraDataFormat);
    priv_lora_net.buffer = (uint8 *)&client_join_data;

    if (AdapterDeviceJoin(adapter, (uint8 *)&priv_lora_net) < 0) {
        return -1;
    }

    return 0;
}

/**
 * @description: Lora Client quit net function
 * @param adapter Lora adapter pointer
 * @param panid Lora net panid
 */
static int LoraClientQuitNet(struct Adapter *adapter, unsigned short panid)
{
    struct LoraDataFormat client_quit_data;
    struct AdapterData priv_lora_net;

    memset(&client_quit_data, 0, sizeof(struct LoraDataFormat));

    client_quit_data.flame_head = ADAPTER_LORA_DATA_HEAD;
    client_quit_data.length = sizeof(struct LoraDataFormat);
    client_quit_data.panid = panid;
    client_quit_data.data_type = ADAPTER_LORA_DATA_TYPE_QUIT;
    client_quit_data.client_id = adapter->net_role_id;
    client_quit_data.crc16 = LoraCrc16((uint8 *)&client_quit_data, sizeof(struct LoraDataFormat) - 2);
    
    priv_lora_net.len = sizeof(struct LoraDataFormat);
    priv_lora_net.buffer = (uint8 *)&client_quit_data;

    if (AdapterDeviceDisconnect(adapter, (uint8 *)&priv_lora_net) < 0) {
        return -1;
    }

    return 0;
}

/**
 * @description: Lora Gateway Process function
 * @param lora_adapter Lora adapter pointer
 * @param gateway Lora gateway pointer
 * @param gateway_recv_data Lora gateway receive data pointer
 */
int LoraGatewayProcess(struct Adapter *lora_adapter, struct LoraGatewayParam *gateway, struct LoraDataFormat *gateway_recv_data)
{
    int i, ret = 0;
    uint8 lora_recv_data[ADAPTER_LORA_RECV_DATA_LENGTH];
    switch (LoraGatewayState)
    {
    case LORA_STATE_IDLE:
        memset(lora_recv_data, 0, ADAPTER_LORA_RECV_DATA_LENGTH);

        ret = AdapterDeviceRecv(lora_adapter, lora_recv_data, ADAPTER_LORA_RECV_DATA_LENGTH);
        if (ret <= 0)  {
            printf("LoraGatewayProcess IDLE recv error.Just return\n");
            break;
        }

        LoraReceiveDataCheck(lora_recv_data, ADAPTER_LORA_RECV_DATA_LENGTH, gateway_recv_data);

        if (ADAPTER_LORA_DATA_TYPE_JOIN == gateway_recv_data->data_type) {
            LoraGatewayState = LORA_JOIN_NET;
        } else if (ADAPTER_LORA_DATA_TYPE_QUIT == gateway_recv_data->data_type) {
            LoraGatewayState = LORA_QUIT_NET;
        } else {
            LoraGatewayState = LORA_STATE_IDLE;
        }
        break;
    case LORA_JOIN_NET:
    case LORA_QUIT_NET:
        ret = LoraGateWayDataAnalyze(lora_adapter, gateway_recv_data);
        if (ret < 0) {
            printf("LoraGateWayDataAnalyze state %d error, re-send data cmd to client\n", LoraGatewayState);
            PrivTaskDelay(500);
        }
        LoraGatewayState = LORA_RECV_DATA;
        break;
    case LORA_RECV_DATA:
        for (i = 0; i < gateway->client_num; i ++) {
            if (gateway->client_id[i]) {
                printf("LoraGatewayProcess send to client %d for data\n", gateway->client_id[i]);
                ret = LoraGatewaySendCmd(lora_adapter, gateway->client_id[i], ADAPTER_LORA_DATA_TYPE_CMD);
                if (ret < 0) {
                    printf("LoraGatewaySendCmd client ID %d error\n", gateway->client_id[i]);
                    PrivTaskDelay(500);
                    continue;
                }

                memset(lora_recv_data, 0, ADAPTER_LORA_RECV_DATA_LENGTH);
                ret = AdapterDeviceRecv(lora_adapter, lora_recv_data, ADAPTER_LORA_RECV_DATA_LENGTH);
                if (ret <= 0) {
                    printf("LoraGatewayProcess recv error.Just return\n");
                    continue;
                }

                LoraReceiveDataCheck(lora_recv_data, ADAPTER_LORA_RECV_DATA_LENGTH, gateway_recv_data);

                if (ADAPTER_LORA_DATA_TYPE_JOIN == gateway_recv_data->data_type) {
                    LoraGatewayState = LORA_JOIN_NET;
                } else if (ADAPTER_LORA_DATA_TYPE_QUIT == gateway_recv_data->data_type) {
                    LoraGatewayState = LORA_QUIT_NET;
                } else {
                    ret = LoraGateWayDataAnalyze(lora_adapter, gateway_recv_data);
                    if (ret < 0) {
                        printf("LoraGateWayDataAnalyze error, re-send data cmd to client\n");
                        PrivTaskDelay(500);
                    }
                }
            }
        }
        break;
    default:
        break;
    }

    return 0;
}

/**
 * @description: Lora Gateway task
 * @param parameter - Lora adapter pointer
 */
static void *LoraGatewayTask(void *parameter)
{
    int i;
    int ret = 0;
    struct Adapter *lora_adapter = (struct Adapter *)parameter;
    struct LoraGatewayParam *gateway = (struct LoraGatewayParam *)lora_adapter->adapter_param;
    struct LoraDataFormat gateway_recv_data;
    
    while (1) {
        LoraGatewayProcess(lora_adapter, gateway, &gateway_recv_data);
    }

    return 0;
}

/**
 * @description: Lora Client data upload task
 * @param parameter - Lora adapter pointer
 */
static void *LoraClientDataTask(void *parameter)
{
    int i, ret = 0;
    struct Adapter *lora_adapter = (struct Adapter *)parameter;
    struct LoraClientParam *client = (struct LoraClientParam *)lora_adapter->adapter_param;

    //set lora_send_buf for test
    uint8 lora_send_buf[ADAPTER_LORA_DATA_LENGTH];
    memset(lora_send_buf, 0, ADAPTER_LORA_DATA_LENGTH);
    sprintf(lora_send_buf, "Lora client %d adapter test\n", client->client_id);

    while (1) {

        PrivTaskDelay(100);

        if ((CLIENT_DISCONNECT == client->client_state) && (!g_adapter_lora_quit_flag)) {
            ret = LoraClientJoinNet(lora_adapter, client->panid);
            if (ret < 0) {
                printf("LoraClientJoinNet error panid 0x%x\n", client->panid);
            }

            ret = LoraClientDataAnalyze(lora_adapter, NULL, 0);
            if (ret < 0) {
                printf("LoraClientDataAnalyze error, reconnect to gateway\n");
                PrivTaskDelay(500);
                continue;
            }
        }

        if (CLIENT_CONNECT == client->client_state) {
            ret = LoraClientDataAnalyze(lora_adapter, (void *)lora_send_buf, strlen(lora_send_buf));
            if (ret < 0) {
                printf("LoraClientDataAnalyze error, wait for next data cmd\n");
                PrivTaskDelay(500);
                continue;
            }
        }
    }

    return 0;
}

/**
 * @description: Lora Client quit task
 * @param parameter - Lora adapter pointer
 */
static void *LoraClientQuitTask(void *parameter)
{
    int ret = 0;
    struct Adapter *lora_adapter = (struct Adapter *)parameter;
    struct LoraClientParam *client = (struct LoraClientParam *)lora_adapter->adapter_param;

    while (1) {
        PrivTaskDelay(100);

        if ((CLIENT_CONNECT == client->client_state) && (g_adapter_lora_quit_flag)) {
            ret = LoraClientQuitNet(lora_adapter, client->panid);
            if (ret < 0) {
                printf("LoraClientQuitNet error panid 0x%x, re-quit net\n", client->panid);
                continue;
            }

            ret = LoraClientDataAnalyze(lora_adapter, NULL, 0);
            if (ret < 0) {
                printf("LoraClientQuitTask LoraClientDataAnalyze error\n");
                PrivTaskDelay(500);
                continue;
            }
        }
    }

    return 0;
}

/*******************LORA ADAPTER FUNCTION********************/

static int AdapterLoraRegister(struct Adapter *adapter)
{
    int ret = 0;
    struct LoraGatewayParam *lora_gateway;
    struct LoraClientParam *lora_client;

    strncpy(adapter->name, ADAPTER_LORA_NAME, NAME_NUM_MAX);

    adapter->net_protocol = PRIVATE_PROTOCOL;

#ifdef AS_LORA_GATEWAY_ROLE
    lora_gateway = PrivMalloc(sizeof(struct LoraGatewayParam));
    if (!lora_gateway) {
        PrivFree(lora_gateway);
        return -1;
    }

    memset(lora_gateway, 0, sizeof(struct LoraGatewayParam));

    lora_gateway->gateway_id = ADAPTER_LORA_NET_ROLE_ID;
    lora_gateway->panid = ADAPTER_LORA_NET_PANID;

    adapter->net_role = GATEWAY;
    adapter->adapter_param = (void *)lora_gateway;
#else //AS_LORA_CLIENT_ROLE
    lora_client = PrivMalloc(sizeof(struct LoraClientParam));
    if (!lora_client) {
        PrivFree(lora_client);
        return -1;
    }

    memset(lora_client, 0, sizeof(struct LoraClientParam));

    lora_client->client_id = ADAPTER_LORA_NET_ROLE_ID;
    lora_client->client_state = CLIENT_DISCONNECT;
    lora_client->panid = ADAPTER_LORA_NET_PANID;

    adapter->net_role = CLIENT;
    adapter->adapter_param = (void *)lora_client;
#endif

    adapter->net_role_id = ADAPTER_LORA_NET_ROLE_ID;

    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0) {
        printf("Adapter4G register error\n");
        if (lora_gateway)
            PrivFree(lora_gateway);
        if (lora_client)
            PrivFree(lora_client);

        return -1;
    }

    return ret;
}

int AdapterLoraInit(void)
{
    int ret = 0;

    struct Adapter *adapter = PrivMalloc(sizeof(struct Adapter));
    if (!adapter) {
        PrivFree(adapter);
        return -1;
    }

    memset(adapter, 0, sizeof(struct Adapter));

    ret = AdapterLoraRegister(adapter);
    if (ret < 0) {
        printf("AdapterLoraInit register lora adapter error\n");
        PrivFree(adapter);
        return -1;
    }

#ifdef ADAPTER_SX1278
    AdapterProductInfoType product_info = Sx1278Attach(adapter);
    if (!product_info) {
        printf("AdapterLoraInit sx1278 attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;

#endif

#ifdef ADAPTER_E220
    AdapterProductInfoType product_info = E220Attach(adapter);
    if (!product_info) {
        printf("AdapterLoraInit e220 attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;
#endif

    return ret;
}

/******************Lora TEST*********************/
#ifdef AS_LORA_GATEWAY_ROLE
static pthread_t lora_gateway_task;
#else //AS_LORA_CLIENT_ROLE
static pthread_t lora_client_data_task;
static pthread_t lora_client_quit_task;
#endif

int AdapterLoraTest(void)
{
    struct Adapter *adapter =  AdapterDeviceFindByName(ADAPTER_LORA_NAME);

    AdapterDeviceOpen(adapter);

    //create lora gateway task
#ifdef AS_LORA_GATEWAY_ROLE
#ifdef ADD_NUTTX_FETURES
    pthread_attr_t lora_gateway_attr = PTHREAD_ATTR_INITIALIZER;
    lora_gateway_attr.priority = 20;
    lora_gateway_attr.stacksize = 2048;
#else
    pthread_attr_t lora_gateway_attr;
    lora_gateway_attr.schedparam.sched_priority = 20;
    lora_gateway_attr.stacksize = 2048;
#endif

    PrivTaskCreate(&lora_gateway_task, &lora_gateway_attr, &LoraGatewayTask, (void *)adapter);
    PrivTaskStartup(&lora_gateway_task);

#else //AS_LORA_CLIENT_ROLE
#ifdef ADD_NUTTX_FETURES
    pthread_attr_t lora_client_attr = PTHREAD_ATTR_INITIALIZER;
    lora_client_attr.priority = 20;
    lora_client_attr.stacksize = 2048;
#else
    pthread_attr_t lora_client_attr;
    lora_client_attr.schedparam.sched_priority = 20;
    lora_client_attr.stacksize = 2048;
#endif
    //create lora client task
    PrivTaskCreate(&lora_client_data_task, &lora_client_attr, &LoraClientDataTask, (void *)adapter);
    PrivTaskStartup(&lora_client_data_task);

    lora_client_attr.stacksize = 1024;

    PrivTaskCreate(&lora_client_quit_task, &lora_client_attr, &LoraClientQuitTask, (void *)adapter);
    PrivTaskStartup(&lora_client_quit_task);
#endif

    return 0;
}
#ifdef ADD_XIZI_FETURES
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, AdapterLoraTest, AdapterLoraTest, show adapter lora information);
#endif

