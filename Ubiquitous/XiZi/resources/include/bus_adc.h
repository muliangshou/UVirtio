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
* @file bus_adc.h
* @brief define adc bus and drv function using bus driver framework
* @version 1.1 
* @author AIIT XUOS Lab
* @date 2021-12-28
*/

#ifndef BUS_ADC_H
#define BUS_ADC_H

#include <bus.h>

#ifdef __cplusplus
extern "C" {
#endif

struct AdcDriver
{
    struct Driver driver;
    uint32 (*configure) (void *drv, struct BusConfigureInfo *configure_info);

    void *private_data;
};

struct AdcBus
{
    struct Bus bus;

    void *private_data;
};

/*Register the ADC bus*/
int AdcBusInit(struct AdcBus *adc_bus, const char *bus_name);

/*Register the ADC driver*/
int AdcDriverInit(struct AdcDriver *adc_driver, const char *driver_name);

/*Release the ADC device*/
int AdcReleaseBus(struct AdcBus *adc_bus);

/*Register the ADC driver to the ADC bus*/
int AdcDriverAttachToBus(const char *drv_name, const char *bus_name);

/*Register the driver, manage with the double linklist*/
int AdcDriverRegister(struct Driver *driver);

/*Find the register driver*/
DriverType AdcDriverFind(const char *drv_name, enum DriverType_e drv_type);

#ifdef __cplusplus
}
#endif

#endif
