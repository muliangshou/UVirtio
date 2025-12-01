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
 * @file plc.c
 * @brief plc relative activities
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.15
 */

#include "ua_api.h"
#include "plc_channel.h"
#include "plc_device.h"

DoublelistType plcdev_list;

/******************************************************************************/

/*Create the plc device linklist*/
static void PlcDeviceLinkInit()
{
    AppInitDoubleList(&plcdev_list);
}

static int PlcDeviceOpen(void *dev)
{
    CHECK_CH_PARAM(dev);

    struct PlcDevice *plc_dev = (struct PlcDevice *)dev;

    if(plc_dev->net == PLC_IND_ENET_OPCUA)
    {
        return UaDevOpen(plc_dev->priv_data);
    }

    return EOK;
}

static void PlcDeviceClose(void *dev)
{
    CHECK_CH_PARAM(dev);

    struct PlcDevice *plc_dev = (struct PlcDevice *)dev;

    if(plc_dev->net == PLC_IND_ENET_OPCUA)
    {
        UaDevClose(plc_dev->priv_data);
    }
}

static int PlcDeviceWrite(void *dev, const void *buf, size_t len)
{
    CHECK_CH_PARAM(dev);

    int ret;
    struct PlcDevice *plc_dev = (struct PlcDevice *)dev;

    if(plc_dev->net == PLC_IND_ENET_OPCUA)
    {
        ret = UaDevWrite(plc_dev->priv_data, buf, len);
    }

    return ret;
}

static int PlcDeviceRead(void *dev, void *buf, size_t len)
{
    CHECK_CH_PARAM(dev);
    CHECK_CH_PARAM(buf);

    int ret;
    struct PlcDevice *plc_dev = (struct PlcDevice *)dev;

    if(plc_dev->net == PLC_IND_ENET_OPCUA)
    {
        ret = UaDevRead(plc_dev->priv_data, buf, len);
    }

    return ret;
}

static struct PlcOps plc_done =
{
    .open = PlcDeviceOpen,
    .close = PlcDeviceClose,
    .write = PlcDeviceWrite,
    .read = PlcDeviceRead,
};

/* find PLC device with device name */
struct ChDev *PlcDevFind(const char *dev_name)
{
    CHECK_CH_PARAM(dev_name);

    struct PlcDevice *device = NONE;
    struct ChDev *haldev = NONE;

    DoublelistType *node = NONE;
    DoublelistType *head = &plcdev_list;

    for (node = head->node_next; node != head; node = node->node_next) {
        device = DOUBLE_LIST_ENTRY(node, struct PlcDevice, link);
        if (!strcmp(device->name, dev_name)) {
            haldev = &device->haldev;
            return haldev;
        }
    }

    plc_print("plc: [%s] cannot find the %s device\n", __func__, dev_name);
    return NONE;
}

int PlcDevRegister(struct PlcDevice *plc_device, void *plc_param, const char *device_name)
{
    CHECK_CH_PARAM(plc_device);
    CHECK_CH_PARAM(device_name);

    int ret = EOK;
    static uint8_t dev_link_flag = 0;

    if (!dev_link_flag) {
        PlcDeviceLinkInit();
        dev_link_flag = 1;
    }

    if (CHDEV_INSTALL != plc_device->state) {
        strncpy(plc_device->haldev.dev_name, device_name, NAME_NUM_MAX);
        plc_device->haldev.dev_type = CHDEV_PLC_TYPE;
        plc_device->haldev.dev_state = CHDEV_INSTALL;

        strncpy(plc_device->name, device_name, strlen(device_name));
        plc_device->ops = &plc_done;

        AppDoubleListInsertNodeAfter(&plcdev_list, &(plc_device->link));
        plc_device->state = CHDEV_INSTALL;

    } else {
        KPrintf("PlcDevRegister device %s has been register state%u\n", device_name, plc_device->state);
        ret = ERROR;
    }

    return ret;
}

int PlcDeviceAttachToChannel(const char *dev_name, const char *ch_name)
{
    CHECK_CH_PARAM(dev_name);
    CHECK_CH_PARAM(ch_name);

    int ret = EOK;

    struct Channel *ch;
    struct ChDev *device;

    ch = ChannelFind(ch_name);
    if (NONE == ch) {
        KPrintf("PlcDeviceAttachToChannel find plc ch error!name %s\n", ch_name);
        return ERROR;
    }

    if (CH_PLC_TYPE == ch->ch_type) {
        device = PlcDevFind(dev_name);
        if (NONE == device) {
            KPrintf("PlcDeviceAttachToChannel find plc device %s error!\n", dev_name);
            return ERROR;
        }

        if (CHDEV_PLC_TYPE == device->dev_type) {
            ret = DeviceRegisterToChannel(ch, device);
            if (EOK != ret) {
                KPrintf("PlcDeviceAttachToChannel DeviceRegisterToChannel error %u\n", ret);
                return ERROR;
            }
        }
    }

    return EOK;
}

