#include <bus_virtio.h>
#include <dev_virtio.h>

int VirtioBusInit(struct VirtioBus *virtio_bus, const char *bus_name)
{
    NULL_PARAM_CHECK(virtio_bus);
    NULL_PARAM_CHECK(bus_name);

    x_err_t ret = EOK;

    if (BUS_INSTALL != virtio_bus->bus.bus_state) {
        strncpy(virtio_bus->bus.bus_name, bus_name, NAME_NUM_MAX);

        virtio_bus->bus.bus_type = TYPE_VIRTIO_BUS;
        virtio_bus->bus.bus_state = BUS_INSTALL;
        virtio_bus->bus.private_data = virtio_bus->private_data;

        ret = BusRegister(&virtio_bus->bus);
        if (EOK != ret) {
            KPrintf("virtio BusInit BusRegister error %u\n", ret);
            return ret;
        }
    } else {
        KPrintf("VirtioBusInit BusRegister bus has been register state%u\n", virtio_bus->bus.bus_state);        
    }

    return ret;
}

int VirtioDriverInit(struct VirtioDriver *virtio_driver, const char *driver_name)
{
    NULL_PARAM_CHECK(virtio_driver);
    NULL_PARAM_CHECK(driver_name);

    x_err_t ret = EOK;

    if (DRV_INSTALL != virtio_driver->driver.driver_state) {
        virtio_driver->driver.driver_type = TYPE_VIRTIO_DRV;
        virtio_driver->driver.driver_state = DRV_INSTALL;

        strncpy(virtio_driver->driver.drv_name, driver_name, NAME_NUM_MAX);

        virtio_driver->driver.configure = virtio_driver->configure;

        ret = VirtioDriverRegister(&virtio_driver->driver);
        if (EOK != ret) {
            KPrintf("VirtioDriverInit DriverRegister error %u\n", ret);
            return ret;
        }
    } else {
        KPrintf("VirtioDriverInit DriverRegister driver has been register state%u\n", virtio_driver->driver.driver_state);
    }

    return ret;
}

int VirtioReleaseBus(struct VirtioBus *virtio_bus)
{
    NULL_PARAM_CHECK(virtio_bus);

    return BusRelease(&virtio_bus->bus);
}

int VirtioDriverAttachToBus(const char *drv_name, const char *bus_name)
{
    NULL_PARAM_CHECK(drv_name);
    NULL_PARAM_CHECK(bus_name);
    
    x_err_t ret = EOK;

    struct Bus *bus;
    struct Driver *driver;

    bus = BusFind(bus_name);
    if (NONE == bus) {
        KPrintf("VirtioDriverAttachToBus find virtio bus error!name %s\n", bus_name);
        return ERROR;
    }

    if (TYPE_VIRTIO_BUS == bus->bus_type) {
        driver = VirtioDriverFind(drv_name, TYPE_VIRTIO_DRV);
        if (NONE == driver) {
            KPrintf("VirtioDriverAttachToBus find virtio driver error!name %s\n", drv_name);
            return ERROR;
        }

        if (TYPE_VIRTIO_DRV == driver->driver_type) {
            ret = DriverRegisterToBus(bus, driver);
            if (EOK != ret) {
                KPrintf("VirtioDriverAttachToBus DriverRegisterToBus error %u\n", ret);
                return ERROR;
            }
        }
    }

    return ret;
}