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
* @file bus_dac.h
* @brief define dac bus and drv function using bus driver framework
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-1-11
*/

#ifndef BUS_DAC_H
#define BUS_DAC_H

#include <bus.h>

#ifdef __cplusplus
extern "C" {
#endif

struct DacDriver
{
    struct Driver driver;
    uint32 (*configure) (void *drv, struct BusConfigureInfo *configure_info);

    void *private_data;
};

struct DacBus
{
    struct Bus bus;

    void *private_data;
};

/*Register the DAC bus*/
int DacBusInit(struct DacBus *dac_bus, const char *bus_name);

/*Register the DAC driver*/
int DacDriverInit(struct DacDriver *dac_driver, const char *driver_name);

/*Release the DAC device*/
int DacReleaseBus(struct DacBus *dac_bus);

/*Register the DAC driver to the DAC bus*/
int DacDriverAttachToBus(const char *drv_name, const char *bus_name);

/*Register the driver, manage with the double linklist*/
int DacDriverRegister(struct Driver *driver);

/*Find the register driver*/
DriverType DacDriverFind(const char *drv_name, enum DriverType_e drv_type);

#ifdef __cplusplus
}
#endif

#endif
