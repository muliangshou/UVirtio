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
* @file dev_dac.c
* @brief register dac dev function using bus driver framework
* @version 1.1 
* @author AIIT XUOS Lab
* @date 2021-12-28
*/

#include <bus_dac.h>
#include <dev_dac.h>

static DoubleLinklistType dacdev_linklist;

/*Create the DAC device linklist*/
static void DacDeviceLinkInit()
{
    InitDoubleLinkList(&dacdev_linklist);
}

/*Find the register DAC device*/
HardwareDevType DacDeviceFind(const char *dev_name, enum DevType dev_type)
{
    NULL_PARAM_CHECK(dev_name);
    
    struct HardwareDev *device = NONE;

    DoubleLinklistType *node = NONE;
    DoubleLinklistType *head = &dacdev_linklist;
    for (node = head->node_next; node != head; node = node->node_next) {    
        device = SYS_DOUBLE_LINKLIST_ENTRY(node, struct HardwareDev, dev_link);
        if ((!strcmp(device->dev_name, dev_name)) && (dev_type == device->dev_type)) {
            return device;
        }
    }

    KPrintf("DacDeviceFind dacnot find the %s device.return NULL\n", dev_name);
    return NONE;
}

/*Register the DAC device*/
int DacDeviceRegister(struct DacHardwareDevice *dac_device, void *dac_param, const char *device_name)
{
    NULL_PARAM_CHECK(dac_device);
    NULL_PARAM_CHECK(device_name);

    x_err_t ret = EOK;    
    static x_bool dev_link_flag = RET_FALSE;

    if (!dev_link_flag) {
        DacDeviceLinkInit();
        dev_link_flag = RET_TRUE;
    }

    if (DEV_INSTALL != dac_device->haldev.dev_state) {
        strncpy(dac_device->haldev.dev_name, device_name, NAME_NUM_MAX);
        dac_device->haldev.dev_type = TYPE_DAC_DEV;
        dac_device->haldev.dev_state = DEV_INSTALL;

        dac_device->haldev.dev_done = (struct HalDevDone *)dac_device->dac_dev_done;

        dac_device->haldev.private_data = dac_param;

        DoubleLinkListInsertNodeAfter(&dacdev_linklist, &(dac_device->haldev.dev_link));
    } else {
        KPrintf("DacDeviceRegister device has been register state%u\n", dac_device->haldev.dev_state);        
    }

    return ret;
}

/*Register the DAC Device to the DAC BUS*/
int DacDeviceAttachToBus(const char *dev_name, const char *bus_name)
{
    NULL_PARAM_CHECK(dev_name);
    NULL_PARAM_CHECK(bus_name);
    
    x_err_t ret = EOK;

    struct Bus *bus;
    struct HardwareDev *device;

    bus = BusFind(bus_name);
    if (NONE == bus) {
        KPrintf("DacDeviceAttachToBus find dac bus error!name %s\n", bus_name);
        return ERROR;
    }

    if (TYPE_DAC_BUS == bus->bus_type) {
        device = DacDeviceFind(dev_name, TYPE_DAC_DEV);

        if (NONE == device) {
            KPrintf("DacDeviceAttachToBus find dac device error!name %s\n", dev_name);
            return ERROR;
        }
    
        if (TYPE_DAC_DEV == device->dev_type) {
            ret = DeviceRegisterToBus(bus, device);

            if (EOK != ret) {
                KPrintf("DacDeviceAttachToBus DeviceRegisterToBus error %u\n", ret);
                return ERROR;
            }
        }
    }

    return EOK;
}
