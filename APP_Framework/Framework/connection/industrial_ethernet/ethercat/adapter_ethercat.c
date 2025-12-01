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
 * @file adapter_ethercat.c
 * @brief Implement the connection ethercat adapter function, reuse the ethernet implementations
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.10.15
 */

#include <adapter.h>
#include <ethercat.h>

#ifdef ADAPTER_HFA21_ETHERCAT
extern AdapterProductInfoType Hfa21EthercatAttach(struct Adapter *adapter);
#endif

/**
 * @description: clear the datagram in ethercat frame, do not free the frame itself, since it may be a memory space on stack
 * @param frame - ethercat frame pointer
 */
void EcatClear(EcatFramePtr frame)
{
    EcatDatagramPtr datagram = frame->datagram;
    while (datagram)
    {
        EcatDatagramPtr temp = datagram->next;
        PrivFree(datagram->data);
        PrivFree(datagram);
        datagram = temp;
    }
    memset(frame, 0, sizeof(EcatFrame));
    frame->header.length = 2; // 2 bytes for the ethercat header
    frame->header.res = 1;
    frame->header.type = 0b1001;
}

/**
 * @description: receive data from adapter
 * @param frame - ethercat frame pointer
 * @param address - ethercat address
 * @param data - data to be carried inside frame
 * @param data_len - length of provided `data`
 * @return success: 0, else return error code.
 */
int EcatAppend(EcatFramePtr frame, EcatAddress address, uint8_t *data, uint16_t data_len)
{
    EcatDatagramPtr datagram = frame->datagram;
    if (!datagram)
    {
        datagram = PrivMalloc(sizeof(EcatDatagram));
        if (!datagram)
        {
            printf("EcatAppend malloc error\n");
            return -1;
        }
        memset(datagram, 0, sizeof(EcatDatagram));
        frame->datagram = datagram;
    }
    else
    {
        while (datagram->next)
        {
            datagram = datagram->next;
        }
        datagram->header.m = 1; // indcate the datagram is not the last one
        datagram->next = PrivMalloc(sizeof(EcatDatagram));
        if (!datagram->next)
        {
            printf("EcatAppend malloc error\n");
            return -1;
        }
        memset(datagram->next, 0, sizeof(EcatDatagram));
        datagram = datagram->next;
    }
    datagram->header.address = address;
    datagram->header.length = data_len;
    datagram->data = data;
    datagram->work_counter++;
    frame->header.length += sizeof(datagram->header) + data_len + sizeof(datagram->work_counter);
    printf("EcatAppend change length to %d\n", frame->header.length);
    return 0;
}

/**
 * @description: find datagram for self and update its content
 * @param frame - ethercat frame pointer
 * @param address - ethercat address
 * @param data - data to be carried inside frame
 * @param data_len - length of provided `data`
 * @return success: 0, else return error code.
 */
int EcatUpdate(EcatFramePtr frame, uint8_t *data, uint16_t data_len)
{
    EcatDatagramPtr datagram = frame->datagram;
    if (!datagram)
    {
        printf("EcatUpdate error: null datagram.\n");
        return -1;
    }
    while (datagram)
    {
        if (datagram->self)
        {
            datagram->header.length = data_len;
            frame->header.length -= strlen(datagram->data);
            datagram->data = data;
            frame->header.length +=  data_len;
            datagram->work_counter++;
            return 0;
        }
        else
        {
            datagram = datagram->next;
        }
    }
    printf("EcatUpdate error: cannot find datagram for node itself.\n");
    return -2;
}

static int AdapterEthercatRegister(struct Adapter *adapter)
{
    int ret = 0;

    strncpy(adapter->name, ADAPTER_ETHERCAT_NAME, NAME_NUM_MAX);

    adapter->net_protocol = IP_PROTOCOL;
    adapter->adapter_status = UNREGISTERED;

    ret = AdapterDeviceRegister(adapter);
    if (ret < 0)
    {
        printf("AdapterEthercat register error\n");
        return -1;
    }

    return ret;
}

int AdapterEthercatInit(void)
{
    int ret = 0;

    struct Adapter *adapter = PrivMalloc(sizeof(struct Adapter));
    if (!adapter)
    {
        printf("AdapterEthercatInit malloc error\n");
        PrivFree(adapter);
        return -1;
    }

    memset(adapter, 0, sizeof(struct Adapter));

    ret = AdapterEthercatRegister(adapter);
    if (ret < 0)
    {
        printf("AdapterEthercatInit register ethercat adapter error\n");
        PrivFree(adapter);
        return -1;
    }

#ifdef ADAPTER_HFA21_ETHERCAT
    AdapterProductInfoType product_info = Hfa21EthercatAttach(adapter);
    if (!product_info)
    {
        printf("AdapterEthercatInit hfa21 attach error\n");
        PrivFree(adapter);
        return -1;
    }

    adapter->product_info_flag = 1;
    adapter->info = product_info;
    adapter->done = product_info->model_done;

#endif

    return ret;
}

/******************ethercat TEST*********************/
extern int Hfa21EthercatSlaveTest(struct Adapter *adapter);
extern int Hfa21EthercatMasterTest(struct Adapter *adapter);

int AdapterEthercatTest(void)
{
    struct Adapter *adapter = AdapterDeviceFindByName(ADAPTER_ETHERCAT_NAME);

#ifdef ADAPTER_HFA21_ETHERCAT
#ifdef AS_ETHERCAT_SLAVE_ROLE
    Hfa21EthercatSlaveTest(adapter);
#else // AS_ETHERCAT_MASTER_ROLE
    Hfa21EthercatMasterTest(adapter);
#endif
#endif

    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_PARAM_NUM(0) | SHELL_CMD_DISABLE_RETURN, AdapterEthercatTest, AdapterEthercatTest, show adapter ethercat information);
