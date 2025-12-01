/**
* @file bus_virtio.h
* @brief define virtio bus and drv function using bus driver framework
* @version 1.0 
* @author Elliot
* @date 2022-03-24
*/

#ifndef BUS_VIRTIO_H
#define BUS_VIRTIO_H

#include <bus.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum ExtSerialPortConfigure
// {
//     PORT_CFG_INIT = 0,
//     PORT_CFG_PARITY_CHECK,
//     PORT_CFG_DISABLE,
//     PORT_CFG_DIV,
// };

// struct SerialDataCfg
// {
//     uint32 serial_baud_rate;
//     uint8 serial_data_bits;
//     uint8 serial_stop_bits;
//     uint8 serial_parity_mode;
//     uint8 serial_bit_order;
//     uint8 serial_invert_mode;
//     uint16 serial_buffer_size;

//     uint8 ext_uart_no;
//     enum ExtSerialPortConfigure port_configure;
// };

// struct SerialHwCfg
// {
//     uint32 serial_register_base;
//     uint32 serial_irq_interrupt;
//     void *private_data;
// };

// struct SerialCfgParam
// {
//     struct SerialDataCfg data_cfg;
//     struct SerialHwCfg hw_cfg;
// };

struct VirtioDriver;

struct VirtioDrvDone
{
    uint32 (*init) (struct VirtioDriver *virtio_drv, struct BusConfigureInfo *configure_info);
    uint32 (*configure) (struct VirtioDriver *virtio_drv, int virtio_operation_cmd);
};

struct VirtioDriver
{
    struct Driver driver;
    const struct VirtioDrvDone *drv_done;

    uint32 (*configure) (void *drv, struct BusConfigureInfo *configure_info);

    void *private_data;
};

struct VirtioBus
{
    struct Bus bus;

    void *private_data;
};

/*Register the virtio bus*/
int VirtioBusInit(struct VirtioBus *virtio_bus, const char *bus_name);

/*Register the virtio driver*/
int VirtioDriverInit(struct VirtioDriver *virtio_driver, const char *driver_name);

/*Release the virtio bus*/
int VirtioReleaseBus(struct VirtioBus *virtio_bus);

/*Register the virtio driver to the virtio bus*/
int VirtioDriverAttachToBus(const char *drv_name, const char *bus_name);

/*Register the driver, manage with the double linklist*/
int VirtioDriverRegister(struct Driver *driver);

/*Find the regiter driver*/
DriverType VirtioDriverFind(const char *drv_name, enum DriverType_e drv_type);

/* Read and check values from MagicValue & Version */
int VirtioDeviceStatusInit(struct Driver *drv);

#ifdef __cplusplus
}
#endif

#endif