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
* @file bus_dac.c
* @brief register dac bus function using bus driver framework
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-1-11
*/

#include <bus_dac.h>
#include <dev_dac.h>

/*Register the DAC BUS*/
int DacBusInit(struct DacBus *dac_bus, const char *bus_name)
{
    NULL_PARAM_CHECK(dac_bus);
    NULL_PARAM_CHECK(bus_name);

    x_err_t ret = EOK;

    if (BUS_INSTALL != dac_bus->bus.bus_state) {
        strncpy(dac_bus->bus.bus_name, bus_name, NAME_NUM_MAX);

        dac_bus->bus.bus_type = TYPE_DAC_BUS;
        dac_bus->bus.bus_state = BUS_INSTALL;
        dac_bus->bus.private_data = dac_bus->private_data;

        ret = BusRegister(&dac_bus->bus);
        if (EOK != ret) {
            KPrintf("DacBusInit BusRegister error %u\n", ret);
            return ret;
        }
    } else {
        KPrintf("DacBusInit BusRegister bus has been register state%u\n", dac_bus->bus.bus_state);        
    }

    return ret;
}

/*Register the DAC Driver*/
int DacDriverInit(struct DacDriver *dac_driver, const char *driver_name)
{
    NULL_PARAM_CHECK(dac_driver);
    NULL_PARAM_CHECK(driver_name);

    x_err_t ret = EOK;

    if (DRV_INSTALL != dac_driver->driver.driver_state) {
        dac_driver->driver.driver_type = TYPE_DAC_DRV;
        dac_driver->driver.driver_state = DRV_INSTALL;

        strncpy(dac_driver->driver.drv_name, driver_name, NAME_NUM_MAX);

        dac_driver->driver.configure = dac_driver->configure;
        dac_driver->driver.private_data = dac_driver->private_data;

        ret = DacDriverRegister(&dac_driver->driver);
        if (EOK != ret) {
            KPrintf("DacDriverInit DriverRegister error %u\n", ret);
            return ret;
        }
    } else {
        KPrintf("DacDriverInit DriverRegister driver has been register state%u\n", dac_driver->driver.driver_state);
    }

    return ret;
}

/*Release the DAC device*/
int DacReleaseBus(struct DacBus *dac_bus)
{
    NULL_PARAM_CHECK(dac_bus);

    return BusRelease(&dac_bus->bus);
}

/*Register the DAC Driver to the DAC BUS*/
int DacDriverAttachToBus(const char *drv_name, const char *bus_name)
{
    NULL_PARAM_CHECK(drv_name);
    NULL_PARAM_CHECK(bus_name);    

    x_err_t ret = EOK;

    struct Bus *bus;
    struct Driver *driver;

    bus = BusFind(bus_name);
    if (NONE == bus) {
        KPrintf("DacDriverAttachToBus find dac bus error!name %s\n", bus_name);
        return ERROR;
    }

    if (TYPE_DAC_BUS == bus->bus_type) {
        driver = DacDriverFind(drv_name, TYPE_DAC_DRV);
        if (NONE == driver) {
            KPrintf("DacDriverAttachToBus find dac driver error!name %s\n", drv_name);
            return ERROR;
        }

        if (TYPE_DAC_DRV == driver->driver_type) {
            ret = DriverRegisterToBus(bus, driver);
            if (EOK != ret) {
                KPrintf("DacDriverAttachToBus DriverRegisterToBus error %u\n", ret);
                return ERROR;
            }
        }
    }

    return ret;
}
