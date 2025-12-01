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
* @file dev_adc.c
* @brief register adc dev function using bus driver framework
* @version 1.1
* @author AIIT XUOS Lab
* @date 2021-12-28
*/

#include <bus_adc.h>
#include <dev_adc.h>

static DoubleLinklistType adcdev_linklist;

/*Create the ADC device linklist*/
static void AdcDeviceLinkInit()
{
    InitDoubleLinkList(&adcdev_linklist);
}

/*Find the register ADC device*/
HardwareDevType AdcDeviceFind(const char *dev_name, enum DevType dev_type)
{
    NULL_PARAM_CHECK(dev_name);

    struct HardwareDev *device = NONE;

    DoubleLinklistType *node = NONE;
    DoubleLinklistType *head = &adcdev_linklist;
    for (node = head->node_next; node != head; node = node->node_next) {
        device = SYS_DOUBLE_LINKLIST_ENTRY(node, struct HardwareDev, dev_link);
        if ((!strcmp(device->dev_name, dev_name)) && (dev_type == device->dev_type)) {
            return device;
        }
    }

    KPrintf("AdcDeviceFind adcnot find the %s device.return NULL\n", dev_name);
    return NONE;
}

/*Register the ADC device*/
int AdcDeviceRegister(struct AdcHardwareDevice *adc_device, void *adc_param, const char *device_name)
{
    NULL_PARAM_CHECK(adc_device);
    NULL_PARAM_CHECK(device_name);

    x_err_t ret = EOK;
    static x_bool dev_link_flag = RET_FALSE;

    if (!dev_link_flag) {
        AdcDeviceLinkInit();
        dev_link_flag = RET_TRUE;
    }

    if (DEV_INSTALL != adc_device->haldev.dev_state) {
        strncpy(adc_device->haldev.dev_name, device_name, NAME_NUM_MAX);
        adc_device->haldev.dev_type = TYPE_ADC_DEV;
        adc_device->haldev.dev_state = DEV_INSTALL;

        adc_device->haldev.dev_done = (struct HalDevDone *)adc_device->adc_dev_done;

        adc_device->haldev.private_data = adc_param;

        DoubleLinkListInsertNodeAfter(&adcdev_linklist, &(adc_device->haldev.dev_link));
    } else {
        KPrintf("AdcDeviceRegister device has been register state%u\n", adc_device->haldev.dev_state);
    }

    return ret;
}

/*Register the ADC Device to the ADC BUS*/
int AdcDeviceAttachToBus(const char *dev_name, const char *bus_name)
{
    NULL_PARAM_CHECK(dev_name);
    NULL_PARAM_CHECK(bus_name);

    x_err_t ret = EOK;

    struct Bus *bus;
    struct HardwareDev *device;

    bus = BusFind(bus_name);
    if (NONE == bus) {
        KPrintf("AdcDeviceAttachToBus find adc bus error!name %s\n", bus_name);
        return ERROR;
    }

    if (TYPE_ADC_BUS == bus->bus_type) {
        device = AdcDeviceFind(dev_name, TYPE_ADC_DEV);

        if (NONE == device) {
            KPrintf("AdcDeviceAttachToBus find adc device error!name %s\n", dev_name);
            return ERROR;
        }

        if (TYPE_ADC_DEV == device->dev_type) {
            ret = DeviceRegisterToBus(bus, device);

            if (EOK != ret) {
                KPrintf("AdcDeviceAttachToBus DeviceRegisterToBus error %u\n", ret);
                return ERROR;
            }
        }
    }

    return EOK;
}

