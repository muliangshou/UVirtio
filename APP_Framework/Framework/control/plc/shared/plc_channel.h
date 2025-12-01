/*
* Copyright (c) 2022 AIIT XUOS Lab
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
* @file plc_channel.h
* @brief define channel driver framework function and common API
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-01
*/

#ifndef __PLC_CHANNEL_H_
#define __PLC_CHANNEL_H_

#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPE_INT                 0x0000
#define OPE_CFG                 0x0001

#define OPER_WDT_SET_TIMEOUT    0x0002
#define OPER_WDT_KEEPALIVE      0x0003

#define CHECK_CH_PARAM(param)            \
do                                     \
{                                      \
    if(param == NONE) {                \
        KPrintf("PARAM CHECK FAILED ...%s %d %s is NULL.\n",__FUNCTION__,__LINE__,#param); \
        while(RET_TRUE);               \
    }                                  \
}while (0)

typedef struct Channel *ChannelType;
typedef struct ChDev *ChDevType;
typedef struct ChDrv *ChDrvType;

/* need to add new ch type in ../tool/shell/letter-shell/cmd.c, ensure ShowBus cmd supported*/
enum ChType_e
{
    CH_PLC_TYPE,
    CH_END_TYPE,
};

enum ChState_e
{
    CHANNEL_INIT = 0,
    CHANNEL_INSTALL,
    CHANNEL_UNINSTALL,
};

enum ChDevType_e
{
    CHDEV_PLC_TYPE,
    CHDEV_END_TYPE,
};

enum ChDevState_e
{
    CHDEV_INIT = 0,
    CHDEV_INSTALL,
    CHDEV_UNINSTALL,
};

enum ChDrvType_e
{
    CHDRV_PLC_TYPE,
    CHDRV_END_TYPE,
};

enum ChDrvState_e
{
    CHDRV_INIT = 0,
    CHDRV_INSTALL,
    CHDRV_UNINSTALL,
};

struct ChConfigInfo
{
    int configure_cmd;
    void *private_data;
};

struct ChReadParam
{
    uint32 pos;
    void* buffer;
    size_t size;
    size_t read_length;
};

struct ChWriteParam
{
    uint32 pos;
    const void* buffer;
    size_t size;
};

struct ChHalDevDone
{
    uint32 (*open) (void *dev);
    uint32 (*close) (void *dev);
    uint32 (*write) (void *dev, struct ChWriteParam *write_param);
    uint32 (*read) (void *dev, struct ChReadParam *read_param);
};

struct ChDev
{
    int8 dev_name[NAME_NUM_MAX];
    enum ChDevType_e dev_type;
    enum ChDevState_e dev_state;

    const struct ChHalDevDone *dev_done;

    int (*dev_recv_callback) (void *dev, size_t length);

    struct Channel *owner_ch;
    void *private_data;

    int32 dev_sem;

    DoublelistType  dev_link;
};

struct ChDrv
{
    int8 drv_name[NAME_NUM_MAX];
    enum ChDrvType_e driver_type;
    enum ChDrvState_e driver_state;

    uint32 (*configure)(void *drv, struct ChConfigInfo *config);

    struct Channel *owner_ch;
    void *private_data;

    DoublelistType  driver_link;
};

struct Channel
{
    int8 ch_name[NAME_NUM_MAX];
    enum ChType_e ch_type;
    enum ChState_e ch_state;

    int32 (*match)(struct ChDrv *driver, struct ChDev *device);

    int ch_lock;

    struct ChDev *owner_haldev;
    struct ChDrv *owner_driver;

    void *private_data;

    /*manage the drv of the channel*/
    uint8 driver_cnt;
    uint8 ch_drvlink_flag;
    DoublelistType ch_drvlink;

    /*manage the dev of the channel*/
    uint8 haldev_cnt;
    uint8 ch_devlink_flag;
    DoublelistType ch_devlink;

    uint8 ch_cnt;
    uint8 ch_link_flag;
    DoublelistType  ch_link;
};

/*Register the BUS,manage with the double linklist*/
int ChannelRegister(struct Channel *ch);

/*Release the BUS framework*/
int ChannelRelease(struct Channel *ch);

/*Unregister a certain kind of BUS*/
int ChannelUnregister(struct Channel *ch);

/*Register the driver to the channel*/
int DriverRegisterToChannel(struct Channel *ch, struct ChDrv *driver);

/*Register the device to the channel*/
int DeviceRegisterToChannel(struct Channel *ch, struct ChDev *device);

/*Delete the driver from the channel*/
int DriverDeleteFromChannel(struct Channel *ch, struct ChDrv *driver);

/*Delete the device from the channel*/
int DeviceDeleteFromChannel(struct Channel *ch, struct ChDev *device);

/*Find the ch with ch name*/
ChannelType ChannelFind(const char *ch_name);

/*Find the driver of cetain channel*/
ChDrvType ChannelFindDriver(struct Channel *ch, const char *driver_name);

/*Find the device of certain channel*/
ChDevType ChannelFindDevice(struct Channel *ch, const char *device_name);

/*Dev receive data callback function*/
uint32 ChannelDevRecvCallback(struct ChDev *dev, int (*dev_recv_callback) (void *dev, size_t length));

/*Open the device of the channel*/
uint32 ChannelDevOpen(struct ChDev *dev);

/*Close the device of the channel*/
uint32 ChannelDevClose(struct ChDev *dev);

/*Write data to the device*/
uint32 ChannelDevWriteData(struct ChDev *dev, struct ChWriteParam *write_param);

/*Read data from the device*/
uint32 ChannelDevReadData(struct ChDev *dev, struct ChReadParam *read_param);

/*Configure the driver of the channel*/
uint32 ChannelDrvConfigure(struct ChDrv *drv, struct ChConfigInfo *config);

/*Obtain the ch using a certain dev*/
int DeviceObtainChannel(struct Channel *ch, struct ChDev *dev, const char *drv_name, struct ChConfigInfo *config);


struct PlcDriver
{
    struct ChDrv driver;
    uint32 (*configure) (void *drv, struct ChConfigInfo *cfg);
};

struct PlcChannel
{
    struct Channel ch;
    void *private_data;
};

/*Register the plc bus*/
int PlcChannelInit(struct PlcChannel *plc_ch, const char *ch_name);

/*Register the plc driver*/
int PlcDriverInit(struct PlcDriver *plc_driver, const char *driver_name);

/*Release the plc device*/
int PlcReleaseChannel(struct PlcChannel *plc_ch);

/*Register the plc driver to the plc bus*/
int PlcDriverAttachToChannel(const char *drv_name, const char *ch_name);

/*Register the driver, manage with the double linklist*/
int PlcDriverRegister(struct ChDrv *driver);

/*Find the register driver*/
ChDrvType PlcDriverFind(const char *drv_name, enum ChDrvType_e drv_type);

#ifdef __cplusplus
}
#endif

#endif
