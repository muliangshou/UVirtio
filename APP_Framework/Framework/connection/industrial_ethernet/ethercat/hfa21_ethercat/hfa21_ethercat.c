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
 * @file hfa21_ethercat.c
 * @brief Implement the connection ethercat adapter function, using HFA21 device
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.10.15
 */

#include <adapter.h>
#include <at_agent.h>
#include <ethercat.h>

// With HFA21, we use transparant transmission mode,
// therefore, only the TCP/UDP datagrams are considered,
// here EtherCAT is in fact an application layer protocol.

// #define ADAPTER_ETHERNET_NAME "ethernet" //"wifi"
EcatFrame ecat_data;
static struct Adapter *ethernet;
uint32_t self_address;

/**
 * @description: Open HFA21 ethernet uart function for ethercat
 * @param adapter - ethercat device pointer
 * @return success: 0, failure: -1
 */
static int Hfa21EthercatOpen(struct Adapter *adapter)
{
    ethernet = AdapterDeviceFindByName(ADAPTER_ETHERNET_NAME);
    AdapterDeviceOpen(ethernet);
    ADAPTER_DEBUG("Hfa21Ethercat open done\n");

    return 0;
}

/**
 * @description: disconnnect ethercat from internet
 * @param adapter - ethercat device pointer
 * @return success: 0
 */
static int Hfa21EthercatSetDown(struct Adapter *adapter)
{
    if (ethernet)
    {
        AdapterDeviceClose(ethernet);
        ethernet = NULL;
    }
    return 0;
}

/**
 * @description: Close HFA21 ethercat and uart function
 * @param adapter - ethercat device pointer
 * @return success: 0
 */
static int Hfa21EthercatClose(struct Adapter *adapter)
{
    return Hfa21EthercatSetDown(adapter);
}

static int Hfa21EthercatBuildAndSend(struct Adapter *adapter)
{
    int result = 0;
    // fill the frame according to the ecat_data
    uint16_t frame_len = sizeof(EcatHeader);
    EcatDatagramPtr cur_data = ecat_data.datagram;
    while (cur_data)
    {
        frame_len += sizeof(EcatDataHeader);
        frame_len += cur_data->header.length;
        frame_len += sizeof(cur_data->work_counter);
        cur_data = (EcatDatagramPtr)cur_data->next;
    }
    if (frame_len > MAX_FRAME_LEN)
    {
        ADAPTER_DEBUG("frame_len is too long\n");
        result = -2;
        goto __exit;
    }
    if (frame_len < MIN_FRAME_LEN)
    {
        ADAPTER_DEBUG("frame_len is too short\n");
        result = -2;
        goto __exit;
    }
    uint8_t *frame = PrivMalloc(sizeof(uint8_t) * frame_len);
    if (frame == NULL)
    {
        ADAPTER_DEBUG("Hfa21EthercatBuildAndSend: malloc failed\n");
        result = -1;
        goto __exit;
    }

    size_t idx = 0;
    WRITE16(frame, idx, ecat_data.header.header);
    cur_data = ecat_data.datagram;
    while (cur_data)
    {
        WRITE8(frame, idx, cur_data->header.cmd);
        WRITE8(frame, idx, cur_data->header.idx);
        WRITE32(frame, idx, cur_data->header.address.logical);
        WRITE16(frame, idx, cur_data->header.suffix);
        WRITE16(frame, idx, cur_data->header.irq);
        memcpy(frame + idx, cur_data->data, cur_data->header.length);
        idx += cur_data->header.length;
        WRITE16(frame, idx, cur_data->work_counter);
        cur_data = (EcatDatagramPtr)cur_data->next;
    }
    // send the frame
    if (ethernet && ethernet->agent)
    {
        result = EntmSend(ethernet->agent, (const char *)frame, frame_len);
        if (result == 0)
        {
            result = frame_len;
        }
    }
    else
    {
        printf("Hfa21Ethercat Send can not find agent!\n");
        result = -1;
        goto __exit;
    }

__exit:
    return result;
}

/**
 * @description: send data to adapter
 * @param adapter - ethercat device pointer
 * @param data - data buffer, when it is NULL, it means to build frame according to `ecat_data`,
 *             otherwise, it means to replace datagram in `ecat_data` and make up a frame. Slave can use
 *            this function to update its data. Master have to construct frame according to `ecat_data` and
 *             pass NULL data to this function.
 * @param data - data length
 * @return success: sent frame len
 */
static int Hfa21EthercatSend(struct Adapter *adapter, const void *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        return Hfa21EthercatBuildAndSend(adapter);
    }
    EcatDatagramPtr cur_data = ecat_data.datagram;
    size_t cur_len = 0;
    while (cur_data)
    {
        if (cur_data->self)
        {
            PrivFree(cur_data->data);
            cur_data->data = PrivMalloc(sizeof(uint8_t) * len);
            memcpy(cur_data->data, data, len);
            cur_data->header.length = len;
            break;
        }
        cur_data = (EcatDatagramPtr)cur_data->next;
    }

    return Hfa21EthercatBuildAndSend(adapter);
}

/**
 * @description: receive data from adapter
 * @param adapter - ethercat device pointer
 * @param data_buffer - data buffer, the whole received frame
 * @param buffer_len - whole frame length, set it to maximum 1500 if you don't know the length
 * @return success: data length, the data can be visited via variable `ecat_data`
 */
static int Hfa21EthercatReceive(struct Adapter *adapter, void *data_buffer, size_t buffer_len)
{
    // in fact, the length of data must be less than buffer_len
    int result = 0;
    if (ethernet && ethernet->agent)
    {
        //
        result = EntmRecv(ethernet->agent, data_buffer, buffer_len, 40000);
    }
    else
    {
        printf("Hfa21Ethercat::Receive can not find agent!\n");
        result = -1;
        goto __exit;
    }
    // parse header
    size_t data_len = 0;
    size_t idx = 0;
    ecat_data.header.header = READ16(data_buffer, idx);
    buffer_len -= ECAT_HEADER_LENGTH;
    if (ecat_data.header.length > buffer_len)
    {
        printf("buffer size is less than the frame length!\n");
        result = -2;
    }
    if (ecat_data.header.length <= ECAT_DATA_HEADER_LENGTH)
    {
        // the first datagram header is too short
        printf("Datagram is empty!\n");
        result = -3;
        goto __exit;
    }

    // parse datagrams
    ecat_data.datagram = (EcatDatagramPtr)PrivMalloc(sizeof(EcatDatagram));
    EcatDatagramPtr cur_data = ecat_data.datagram;
    do
    {
        // parse datagram header
        cur_data->header.cmd = READ8(data_buffer, idx);
        cur_data->header.idx = READ8(data_buffer, idx);
        cur_data->header.address.logical = READ32(data_buffer, idx);
        if (cur_data->header.address.logical == self_address)
        {
            cur_data->self = 1; // this datagram is mine
        }
        else
        {
            cur_data->self = 0;
        }
        cur_data->header.suffix = READ16(data_buffer, idx);
        cur_data->header.irq = READ16(data_buffer, idx);
        if (idx + cur_data->header.length + sizeof(cur_data->work_counter) > buffer_len)
        {
            printf("buffer size is less than the frame length!\n");
            result = -2;
            goto __exit;
        }
        // parse datagram data
        if (cur_data->data)
        {
            PrivFree(cur_data->data);
        }
        cur_data->data = PrivMalloc(sizeof(char) * cur_data->header.length);
        memcpy(cur_data->data, data_buffer + idx, cur_data->header.length);
        idx += cur_data->header.length;
        memcpy(data_buffer + data_len, cur_data->data, cur_data->header.length);
        data_len += cur_data->header.length;
        cur_data->work_counter = READ16(data_buffer, idx);
        // check header flag `more` for next datagram, move to next datagram if exists
        if (cur_data->header.m == 1 && buffer_len >= idx + ECAT_DATA_HEADER_LENGTH)
        {
            if (!cur_data->next)
            {
                cur_data->next = PrivMalloc(sizeof(EcatDatagram));
            }
            cur_data = (EcatDatagramPtr)cur_data->next;
        }
        else
        {
            cur_data = cur_data->next;
            while (cur_data)
            {
                EcatDatagramPtr temp = (EcatDatagramPtr)cur_data->next;
                PrivFree(cur_data->data);
                PrivFree(cur_data);
                cur_data = temp;
            }
            break;
        }
    } while (1);
    result = data_len;
__exit:
    return result;
}

/**
 * @description: connnect Ethercat to internet
 * @param adapter - Ethercat device pointer
 * @return success: 0
 */
static int Hfa21EthercatSetUp(struct Adapter *adapter)
{
    if (ethernet)
    {
        return AdapterDeviceSetUp(ethernet);
    }
    return -1;
}

/**
 * @description: set ethercat ip/gateway/netmask address(in sta mode) working at WANN mode
 * @param adapter - ethercat device pointer
 * @param ip - ip address
 * @param gateway - gateway address
 * @param netmask - netmask address
 * @return success: 0, failure: -ENOMEMORY or -1
 */
static int Hfa21EthercatSetAddr(struct Adapter *adapter, const char *ip, const char *gateway, const char *netmask)
{
    if (ethernet)
    {
        return AdapterDeviceSetAddr(ethernet,
                                    ip == NULL ? ADAPTER_ETHERCAT_HFA21_IP_SELF : ip,
                                    gateway == NULL ? ADAPTER_ETHERCAT_HFA21_GATEWAY : gateway,
                                    netmask == NULL ? ADAPTER_ETHERCAT_HFA21_NETMASK : netmask);
    }
    return -1;
}

/**
 * @description: ethercat ping function
 * @param adapter - ethercat device pointer
 * @param destination - domain name or ip address
 * @return success: 0, failure: -1
 */
static int Hfa21EthercatPing(struct Adapter *adapter, const char *destination)
{
    if (ethernet)
    {
        return AdapterDevicePing(ethernet, destination);
    }
    return -1;
}

/**
 * @description: ethercat connect function
 * @param adapter - ethercat device pointer
 * @param net_role - net role, CLIENT or SERVER
 * @param ip - ip address
 * @param port - port num
 * @param ip_type - ip type, IPV4 or IPV6
 * @return success: 0, failure: -1
 */
static int Hfa21EthercatConnect(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type)
{

    if (ethernet)
    {
        return AdapterDeviceConnect(ethernet, net_role, ip,
                                    port == NULL ? ETHERCAT_PORT : port, ip_type);
    }
    return -1;
}

static int Hfa21EthercatIoctl(struct Adapter *adapter, int cmd, void *args)
{
    if (ethernet)
    {
        return AdapterDeviceControl(ethernet, cmd, args);
    }
    return -1;
}

static const struct IpProtocolDone hfa21_ethercat_done =
    {
        .open = Hfa21EthercatOpen,
        .close = Hfa21EthercatClose,
        .ioctl = Hfa21EthercatIoctl,
        .setup = Hfa21EthercatSetUp,
        .setdown = Hfa21EthercatSetDown,
        .setaddr = Hfa21EthercatSetAddr,
        .setdns = NULL,
        .setdhcp = NULL,
        .ping = Hfa21EthercatPing,
        .netstat = NULL,
        .connect = Hfa21EthercatConnect,
        .send = Hfa21EthercatSend,
        .recv = Hfa21EthercatReceive,
        .disconnect = NULL,
};

/**
 * @description: Register ethercat device hfa21
 * @return success: product_info, failure: NULL
 */
AdapterProductInfoType Hfa21EthercatAttach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = PrivMalloc(sizeof(struct AdapterProductInfo));
    if (!product_info)
    {
        printf("Hfa21EthercatAttach Attach malloc product_info error\n");
        PrivFree(product_info);
        return NULL;
    }
    strcpy(product_info->model_name, ADAPTER_ETHERCAT_HFA21);
    product_info->model_done = (void *)&hfa21_ethercat_done;
    printf("str address during init: %s\n", ADAPTER_ETHERCAT_ADDRESS_SELF);
    self_address = strtoul(ADAPTER_ETHERCAT_ADDRESS_SELF, NULL, 16);
    printf("self address during init: %x\n", self_address);
    return product_info;
}

/**
 * @description: Test case for slave ethercat device for hfa21
 * @return success: 0, failure: -1
 */
int Hfa21EthercatSlaveTest(struct Adapter *adapter)
{
    int baud_rate = BAUD_RATE_57600;
    char ethercat_recv_msg[128] = {0};
    int i, len = 0;
    AdapterDeviceOpen(adapter);
    AdapterDeviceControl(adapter, OPE_INT, &baud_rate);
    AdapterDeviceSetUp(adapter);
    // AdapterDeviceSetAddr(adapter, NULL, NULL, NULL);

    enum IpType ip_type = IPV4;

    printf("ready to test data transfer\n");

    // for slave nodes, receive first, change the contents and then send it to next node
    len = 1500;
    for (i = 0; i < 10; i++)
    {
        // wait for neighbor node to send data
        const char *ip_from = ADAPTER_ETHERCAT_HFA21_IP_FROM;
        AdapterDeviceConnect(adapter, SERVER, ip_from, NULL, ip_type);
        PrivTaskDelay(200);
        AdapterDeviceRecv(adapter, ethercat_recv_msg, 1500);
        printf("AdapterEthercatTest recv %s\n", ethercat_recv_msg);
        memset(ethercat_recv_msg, 0, 128);
        PrivTaskDelay(1000);
        // send processed data to next node
        const char *ip_to = ADAPTER_ETHERCAT_HFA21_IP_TO;
        AdapterDeviceConnect(adapter, CLIENT, ip_to, NULL, ip_type);
        PrivTaskDelay(200);
        printf("AdapterEthercatTest send\n");
        AdapterDeviceSend(adapter, NULL, len);
        PrivTaskDelay(1000);
    }
}

/**
 * @description: Test case for master ethercat device for hfa21
 * @return success: 0, failure: -1
 */
int Hfa21EthercatMasterTest(struct Adapter *adapter)
{
    int baud_rate = BAUD_RATE_57600;
    char ethercat_recv_msg[128] = {0};
    int i, len = 0;
    AdapterDeviceOpen(adapter);
    AdapterDeviceControl(adapter, OPE_INT, &baud_rate);
    AdapterDeviceSetUp(adapter);
    // AdapterDeviceSetAddr(adapter, NULL, NULL, NULL);
    // printf("setup addr\n");

    enum IpType ip_type = IPV4;
    // for master, manually build ethercat frame
    EcatClear(&ecat_data);
    printf("start build first block\n");
    EcatAddress slave1;
    slave1.logical = 0x01020304;
    char data1[] = "cats";
    EcatAppend(&ecat_data, slave1, data1, sizeof(data1));
    printf("start build second block\n");
    EcatAddress slave2;
    slave2.logical = 0x20304050;
    char data2[] = "dog";
    EcatAppend(&ecat_data, slave2, data2, sizeof(data2));

    printf("ready to test data transfer\n");

    len = 1500;
    for (i = 0; i < 10; i++)
    {
        // send a frame to a slave node
        const char *ip_to = ADAPTER_ETHERCAT_HFA21_IP_TO;
        AdapterDeviceConnect(adapter, CLIENT, ip_to, NULL, ip_type);
        PrivTaskDelay(200);
        printf("[AdapterEthercatTest send]\n");
        AdapterDeviceSend(adapter, NULL, len);
        PrivTaskDelay(1000);
        // wait for slaves' responses
        const char *ip_from = ADAPTER_ETHERCAT_HFA21_IP_FROM;
        AdapterDeviceConnect(adapter, SERVER, ip_from, NULL, ip_type);
        PrivTaskDelay(200);
        AdapterDeviceRecv(adapter, ethercat_recv_msg, 1500);
        printf("[AdapterEthercatTest recv] %s\n", ethercat_recv_msg);
        memset(ethercat_recv_msg, 0, 128);
        PrivTaskDelay(1000);
    }
}