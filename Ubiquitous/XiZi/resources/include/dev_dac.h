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
* @file dev_dac.h
* @brief define dac dev function using bus driver framework
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-1-11
*/

#ifndef DEV_DAC_H
#define DEV_DAC_H

#include <bus.h>

#ifdef __cplusplus
extern "C" {
#endif

struct DacHardwareDevice;

struct DacDevDone
{
    uint32 (*open) (void *dev);
    uint32 (*close) (void *dev);
    uint32 (*write) (void *dev,struct BusBlockWriteParam *write_param);
    uint32 (*read) (void *dev, struct BusBlockReadParam *read_param);
};

struct DacHardwareDevice
{
    struct HardwareDev haldev;
    const struct DacDevDone *dac_dev_done;

    void *private_data;
};

/*Register the DAC device*/
int DacDeviceRegister(struct DacHardwareDevice *dac_device, void *dac_param, const char *device_name);

/*Register the DAC device to the DAC bus*/
int DacDeviceAttachToBus(const char *dev_name, const char *bus_name);

/*Find the register DAC device*/
HardwareDevType DacDeviceFind(const char *dev_name, enum DevType dev_type);

#ifdef __cplusplus
}
#endif

#endif
