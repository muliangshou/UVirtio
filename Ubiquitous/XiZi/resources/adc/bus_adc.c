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
* @file bus_adc.c
* @brief register adc bus function using bus driver framework
* @version 1.1
* @author AIIT XUOS Lab
* @date 2021-12-28
*/

#include <bus_adc.h>
#include <dev_adc.h>

/*Register the ADC BUS*/
int AdcBusInit(struct AdcBus *adc_bus, const char *bus_name)
{
    NULL_PARAM_CHECK(adc_bus);
    NULL_PARAM_CHECK(bus_name);

    x_err_t ret = EOK;

    if (BUS_INSTALL != adc_bus->bus.bus_state) {
        strncpy(adc_bus->bus.bus_name, bus_name, NAME_NUM_MAX);

        adc_bus->bus.bus_type = TYPE_ADC_BUS;
        adc_bus->bus.bus_state = BUS_INSTALL;
        adc_bus->bus.private_data = adc_bus->private_data;

        ret = BusRegister(&adc_bus->bus);
        if (EOK != ret) {
            KPrintf("AdcBusInit BusRegister error %u\n", ret);
            return ret;
        }
    } else {
        KPrintf("AdcBusInit BusRegister bus has been register state%u\n", adc_bus->bus.bus_state);
    }

    return ret;
}

/*Register the ADC Driver*/
int AdcDriverInit(struct AdcDriver *adc_driver, const char *driver_name)
{
    NULL_PARAM_CHECK(adc_driver);
    NULL_PARAM_CHECK(driver_name);

    x_err_t ret = EOK;

    if (DRV_INSTALL != adc_driver->driver.driver_state) {
        adc_driver->driver.driver_type = TYPE_ADC_DRV;
        adc_driver->driver.driver_state = DRV_INSTALL;

        strncpy(adc_driver->driver.drv_name, driver_name, NAME_NUM_MAX);

        adc_driver->driver.configure = adc_driver->configure;
        adc_driver->driver.private_data = adc_driver->private_data;

        ret = AdcDriverRegister(&adc_driver->driver);
        if (EOK != ret) {
            KPrintf("AdcDriverInit DriverRegister error %u\n", ret);
            return ret;
        }
    } else {
        KPrintf("AdcDriverInit DriverRegister driver has been register state%u\n", adc_driver->driver.driver_state);
    }

    return ret;
}

/*Release the ADC device*/
int AdcReleaseBus(struct AdcBus *adc_bus)
{
    NULL_PARAM_CHECK(adc_bus);

    return BusRelease(&adc_bus->bus);
}

/*Register the ADC Driver to the ADC BUS*/
int AdcDriverAttachToBus(const char *drv_name, const char *bus_name)
{
    NULL_PARAM_CHECK(drv_name);
    NULL_PARAM_CHECK(bus_name);

    x_err_t ret = EOK;

    struct Bus *bus;
    struct Driver *driver;

    bus = BusFind(bus_name);
    if (NONE == bus) {
        KPrintf("AdcDriverAttachToBus find adc bus error!name %s\n", bus_name);
        return ERROR;
    }

    if (TYPE_ADC_BUS == bus->bus_type) {
        driver = AdcDriverFind(drv_name, TYPE_ADC_DRV);
        if (NONE == driver) {
            KPrintf("AdcDriverAttachToBus find adc driver error!name %s\n", drv_name);
            return ERROR;
        }

        if (TYPE_ADC_DRV == driver->driver_type) {
            ret = DriverRegisterToBus(bus, driver);
            if (EOK != ret) {
                KPrintf("AdcDriverAttachToBus DriverRegisterToBus error %u\n", ret);
                return ERROR;
            }
        }
    }

    return ret;
}

