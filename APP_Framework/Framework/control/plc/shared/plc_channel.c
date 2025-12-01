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
* @file plc_ch.c
* @brief Support channel driver framework provide ch API version 1.0
* @author AIIT XUOS Lab
* @date 2021-04-24
*/

#include "string.h"
#include "plc_channel.h"
#include "plc_device.h"
#include "transform.h"

DoublelistType ch_linklist;

/*Create the ch linklist*/
static void ChannelLinkInit(struct Channel *ch)
{
    static uint8 ch_link_flag = RET_FALSE;

    if(!ch_link_flag) {
        AppInitDoubleList(&ch_linklist);
        ch_link_flag = RET_TRUE;
        ch->ch_link_flag = RET_TRUE;
    }

    /*Create the driver of the ch linklist*/
    if(!ch->ch_drvlink_flag) {
        AppInitDoubleList(&ch->ch_drvlink);
        ch->ch_drvlink_flag = RET_TRUE;
    }

    /*Create the hardware device of the ch linklist*/
    if(!ch->ch_devlink_flag) {
        AppInitDoubleList(&ch->ch_devlink);
        ch->ch_devlink_flag = RET_TRUE;
    }
}

static int ChannelMatchDrvDev(struct ChDrv *driver, struct ChDev *device)
{
    CHECK_CH_PARAM(driver);
    CHECK_CH_PARAM(device);

    if(!strncmp(driver->owner_ch->ch_name, device->owner_ch->ch_name, NAME_NUM_MAX)) {
        KPrintf("ChannelMatchDrvDev match successfully, ch name %s\n", driver->owner_ch->ch_name);

        driver->private_data = device->private_data;//driver get the device param
        device->owner_ch->owner_driver = driver;
        driver->owner_ch->owner_haldev = device;

        return EOK;
    }

    return ERROR;
}

/**
* @Description: support to obtain ch for a certain dev if necessary, then configure and init its drv
* @param ch - ch pointer
* @param dev - dev pointer
* @param drv_name - drv name
* @param config - ChConfigInfo pointer
* @return successful:EOK,failed:ERROR
*/
int DeviceObtainChannel(struct Channel *ch, struct ChDev *dev, const char *drv_name, struct ChConfigInfo *cfg)
{
    CHECK_CH_PARAM(ch);
    CHECK_CH_PARAM(dev);

    int32 ret = EOK;

    ret = PrivMutexObtain(&ch->ch_lock);
    if(EOK != ret) {
        KPrintf("DevObtainChannel ch_lock error %d!\n", ret);
        return ret;
    }

    if(ch->owner_haldev != dev) {
        struct ChDrv *drv = ChannelFindDriver(ch, drv_name);

        cfg->configure_cmd = OPE_CFG;
        drv->configure(drv, cfg);

        cfg->configure_cmd = OPE_INT;
        drv->configure(drv, cfg);

        ch->owner_haldev = dev;
    }

    return ret;
}

/**
* @Description: support to register ch pointer with linklist
* @param ch - ch pointer
* @return successful:EOK,failed:NONE
*/
int ChannelRegister(struct Channel *ch)
{
    int ret = EOK;
    CHECK_CH_PARAM(ch);

    ch->match = ChannelMatchDrvDev;

    ChannelLinkInit(ch);

    PrivMutexCreate(&ch->ch_lock, NULL);

    AppDoubleListInsertNodeAfter(&ch_linklist, &(ch->ch_link));

    return ret;
}

/**
* @Description: support to release ch pointer in linklist
* @param ch - ch pointer
* @return successful:EOK,failed:NONE
*/
int ChannelRelease(struct Channel *ch)
{
    CHECK_CH_PARAM(ch);

    PrivMutexAbandon(&ch->ch_lock);

    ch->ch_cnt = 0;
    ch->driver_cnt = 0;
    ch->haldev_cnt = 0;

    ch->ch_link_flag = RET_FALSE;
    ch->ch_drvlink_flag = RET_FALSE;
    ch->ch_devlink_flag = RET_FALSE;

    return EOK;
}

/**
* @Description: support to unregister ch pointer and delete its linklist node
* @param ch - ch pointer
* @return successful:EOK,failed:NONE
*/
int ChannelUnregister(struct Channel *ch)
{
    CHECK_CH_PARAM(ch);

    ch->ch_cnt--;

    AppDoubleListRmNode(&(ch->ch_link));

    return EOK;
}

/**
* @Description: support to register driver pointer to ch pointer
* @param ch - ch pointer
* @param driver - driver pointer
* @return successful:EOK,failed:NONE
*/
int DriverRegisterToChannel(struct Channel *ch, struct ChDrv *driver)
{
    CHECK_CH_PARAM(ch);
    CHECK_CH_PARAM(driver);

    driver->owner_ch = ch;
    ch->driver_cnt++;

    AppDoubleListInsertNodeAfter(&ch->ch_drvlink, &(driver->driver_link));

    return EOK;
}

/**
* @Description: support to register dev pointer to ch pointer
* @param ch - ch pointer
* @param device - device pointer
* @return successful:EOK,failed:NONE
*/
int DeviceRegisterToChannel(struct Channel *ch, struct ChDev *device)
{
    CHECK_CH_PARAM(ch);
    CHECK_CH_PARAM(device);

    device->owner_ch = ch;
    ch->haldev_cnt++;

    AppDoubleListInsertNodeAfter(&ch->ch_devlink, &(device->dev_link));

    return EOK;
}

/**
* @Description: support to delete driver pointer from ch pointer
* @param ch - ch pointer
* @param driver - driver pointer
* @return successful:EOK,failed:NONE
*/
int DriverDeleteFromChannel(struct Channel *ch, struct ChDrv *driver)
{
    CHECK_CH_PARAM(ch);
    CHECK_CH_PARAM(driver);

    ch->driver_cnt--;

    AppDoubleListRmNode(&(driver->driver_link));

    free(driver);

    return EOK;
}

/**
* @Description: support to delete dev pointer from ch pointer
* @param ch - ch pointer
* @param device - device pointer
* @return successful:EOK,failed:NONE
*/
int DeviceDeleteFromChannel(struct Channel *ch, struct ChDev *device)
{
    CHECK_CH_PARAM(ch);
    CHECK_CH_PARAM(device);

    ch->haldev_cnt--;

    AppDoubleListRmNode(&(device->dev_link));

    free(device);

    return EOK;
}

/**
* @Description: support to find ch pointer by ch name
* @param ch_name - ch name
* @return successful:ch pointer,failed:NONE
*/
ChannelType ChannelFind(const char *ch_name)
{
    struct Channel *ch = NONE;

    DoublelistType *node = NONE;
    DoublelistType *head = &ch_linklist;

    for (node = head->node_next; node != head; node = node->node_next)
    {
        ch = DOUBLE_LIST_ENTRY(node, struct Channel, ch_link);
        if(!strcmp(ch->ch_name, ch_name)) {
            return ch;
        }
    }

    KPrintf("ChannelFind cannot find the %s ch.return NULL\n", ch_name);
    return NONE;
}

/**
* @Description: support to find driver pointer of certain ch by driver name
* @param ch - ch pointer
* @param driver_name - driver name
* @return successful:EOK,failed:NONE
*/
ChDrvType ChannelFindDriver(struct Channel *ch, const char *driver_name)
{
    CHECK_CH_PARAM(ch);
    struct ChDrv *driver = NONE;

    DoublelistType *node = NONE;
    DoublelistType *head = &ch->ch_drvlink;

    for (node = head->node_next; node != head; node = node->node_next)
    {
        driver = DOUBLE_LIST_ENTRY(node, struct ChDrv, driver_link);
        if(!strcmp(driver->drv_name, driver_name)) {
            return driver;
        }
    }

    KPrintf("ChannelFindDriver cannot find the %s driver.return NULL\n", driver_name);
    return NONE;
}

/**
* @Description: support to find device pointer of certain ch by device name
* @param ch - ch pointer
* @param device_name - device name
* @return successful:EOK,failed:NONE
*/
ChDevType ChannelFindDevice(struct Channel *ch, const char *device_name)
{
    CHECK_CH_PARAM(ch);
    struct ChDev *device = NONE;

    DoublelistType *node = NONE;
    DoublelistType *head = &ch->ch_devlink;

    for (node = head->node_next; node != head; node = node->node_next)
    {
        device = DOUBLE_LIST_ENTRY(node, struct ChDev, dev_link);

        if(!strcmp(device->dev_name, device_name)) {
            return device;
        }
    }

    KPrintf("ChannelFindDevice cannot find the %s device.return NULL\n", device_name);
    return NONE;
}

/**
* @Description: support to set dev receive function callback
* @param dev - dev pointer
* @param dev_recv_callback - callback function
* @return successful:EOK,failed:ERROR
*/
uint32 ChannelDevRecvCallback(struct ChDev *dev, int (*dev_recv_callback) (void *dev, size_t length))
{
    CHECK_CH_PARAM(dev );

    dev->dev_recv_callback = dev_recv_callback;

    return EOK;
}

/**
* @Description: support to open dev
* @param dev - dev pointer
* @return successful:EOK,failed:ERROR
*/
uint32 ChannelDevOpen(struct ChDev *dev)
{
    CHECK_CH_PARAM(dev);

    int ret = EOK;

    if (dev->dev_done->open) {
        ret = dev->dev_done->open(dev);
        if(ret) {
            KPrintf("ChannelDevOpen error ret %u\n", ret);
            return ERROR;
        }
    }

    return ret;
}

/**
* @Description: support to close dev
* @param dev - dev pointer
* @return successful:EOK,failed:ERROR
*/
uint32 ChannelDevClose(struct ChDev *dev)
{
    CHECK_CH_PARAM(dev);

    int ret = EOK;

    if (dev->dev_done->close) {
        ret = dev->dev_done->close(dev);
        if(ret) {
            KPrintf("ChannelDevClose error ret %u\n", ret);
            return ERROR;
        }
    }

    return ret;
}

/**
* @Description: support to write data to dev
* @param dev - dev pointer
* @param write_param - ChWriteParam
* @return successful:EOK,failed:NONE
*/
uint32 ChannelDevWriteData(struct ChDev *dev, struct ChWriteParam *write_param)
{
    CHECK_CH_PARAM(dev);

    if (dev->dev_done->write) {
        return dev->dev_done->write(dev, write_param);
    }

    return EOK;
}

/**
* @Description: support to read data from dev
* @param dev - dev pointer
* @param read_param - ChReadParam
* @return successful:EOK,failed:NONE
*/
uint32 ChannelDevReadData(struct ChDev *dev, struct ChReadParam *read_param)
{
    CHECK_CH_PARAM(dev);

    if (dev->dev_done->read) {
        return dev->dev_done->read(dev, read_param);
    }

    return EOK;
}

/**
* @Description: support to configure drv, include OPE_CFG and OPE_INT
* @param drv - drv pointer
* @param config - ChConfigInfo
* @return successful:EOK,failed:NONE
*/
uint32 ChannelDrvConfigure(struct ChDrv *drv, struct ChConfigInfo *config)
{
    CHECK_CH_PARAM(drv);
    CHECK_CH_PARAM(config);

    int ret = EOK;

    if (drv->configure) {
        ret = drv->configure(drv, config);
        if(ret) {
            KPrintf("ChannelDrvCfg error, ret %u\n", ret);
            return ERROR;
        }
    }

    return ret;
}

int PlcChannelInit(struct PlcChannel* plc_ch, const char* ch_name)
{
    CHECK_CH_PARAM(plc_ch);
    CHECK_CH_PARAM(ch_name);
    int ret = EOK;

    if(CHANNEL_INSTALL != plc_ch->ch.ch_state)
    {
        strncpy(plc_ch->ch.ch_name, ch_name, NAME_NUM_MAX);
        plc_ch->ch.ch_type = CH_PLC_TYPE;
        plc_ch->ch.ch_state = CHANNEL_INSTALL;
        plc_ch->ch.private_data = plc_ch->private_data;
        ret = ChannelRegister(&plc_ch->ch);

        if(EOK != ret)
        {
            KPrintf("PlcChannelInit ChannelRegister error %u\n", ret);
            return ret;
        }
    }
    else
    {
        KPrintf("PlcChannelInit ChannelRegister channel has been register state %u\n",
                plc_ch->ch.ch_state);
    }

    return ret;
}

int PlcDriverInit(struct PlcDriver* plc_driver, const char* driver_name)
{
    CHECK_CH_PARAM(plc_driver);
    CHECK_CH_PARAM(driver_name);
    int ret = EOK;

    if(CHDRV_INSTALL != plc_driver->driver.driver_state)
    {
        plc_driver->driver.driver_type = CHDRV_PLC_TYPE;
        plc_driver->driver.driver_state = CHDRV_INSTALL;
        strncpy(plc_driver->driver.drv_name, driver_name, NAME_NUM_MAX);
        plc_driver->driver.configure = plc_driver->configure;
        ret = PlcDriverRegister(&plc_driver->driver);

        if(EOK != ret)
        {
            KPrintf("PlcDriverInit DriverRegister error %u\n", ret);
            return ret;
        }
    }
    else
    {
        KPrintf("PlcDriverInit Driver %s has been register state %u\n",
                driver_name, plc_driver->driver.driver_state);
    }

    return ret;
}

int PlcReleaseChannel(struct PlcChannel* plc_ch)
{
    CHECK_CH_PARAM(plc_ch);
    return ChannelRelease(&plc_ch->ch);
}

int PlcDriverAttachToChannel(const char* drv_name, const char* ch_name)
{
    CHECK_CH_PARAM(drv_name);
    CHECK_CH_PARAM(ch_name);
    int ret = EOK;
    struct Channel* ch;
    struct ChDrv* driver;
    ch = ChannelFind(ch_name);

    if(NONE == ch)
    {
        KPrintf("PlcDriverAttachToChannel find plc channel error!name %s\n", ch_name);
        return ERROR;
    }

    if(CH_PLC_TYPE == ch->ch_type)
    {
        driver = PlcDriverFind(drv_name, CHDRV_PLC_TYPE);

        if(NONE == driver)
        {
            KPrintf("PlcDriverAttachToChannel find plc driver error!name %s\n", drv_name);
            return ERROR;
        }

        if(CHDRV_PLC_TYPE == driver->driver_type)
        {
            ret = DriverRegisterToChannel(ch, driver);

            if(EOK != ret)
            {
                KPrintf("PlcDriverAttachToChannel DriverRegisterToBus error %u\n", ret);
                return ERROR;
            }
        }
    }

    return ret;
}
