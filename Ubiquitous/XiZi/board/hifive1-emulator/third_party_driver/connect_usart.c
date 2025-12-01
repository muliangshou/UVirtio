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
* @file connect_usart.c
* @brief support aiit-arm32-board usart function and register to bus framework
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-04-25
*/

#include <xizi.h>
#include <device.h>

#include <encoding.h>
#include <platform.h>
#include <arch_interrupt.h>
#include <uart.h>
#include <board.h>

#include <dev_virtio.h>
#include <bus_virtio.h>
#include <virtio.h>

static void SerialCfgParamCheck(struct SerialCfgParam *serial_cfg_default, struct SerialCfgParam *serial_cfg_new)
{
    struct SerialDataCfg *data_cfg_default = &serial_cfg_default->data_cfg;
    struct SerialDataCfg *data_cfg_new = &serial_cfg_new->data_cfg;

    if ((data_cfg_default->serial_baud_rate != data_cfg_new->serial_baud_rate) && (data_cfg_new->serial_baud_rate)) {
        data_cfg_default->serial_baud_rate = data_cfg_new->serial_baud_rate;
    }

    if ((data_cfg_default->serial_bit_order != data_cfg_new->serial_bit_order) && (data_cfg_new->serial_bit_order)) {
        data_cfg_default->serial_bit_order = data_cfg_new->serial_bit_order;
    }

    if ((data_cfg_default->serial_buffer_size != data_cfg_new->serial_buffer_size) && (data_cfg_new->serial_buffer_size)) {
        data_cfg_default->serial_buffer_size = data_cfg_new->serial_buffer_size;
    }

    if ((data_cfg_default->serial_data_bits != data_cfg_new->serial_data_bits) && (data_cfg_new->serial_data_bits)) {
        data_cfg_default->serial_data_bits = data_cfg_new->serial_data_bits;
    }

    if ((data_cfg_default->serial_invert_mode != data_cfg_new->serial_invert_mode) && (data_cfg_new->serial_invert_mode)) {
        data_cfg_default->serial_invert_mode = data_cfg_new->serial_invert_mode;
    }

    if ((data_cfg_default->serial_parity_mode != data_cfg_new->serial_parity_mode) && (data_cfg_new->serial_parity_mode)) {
        data_cfg_default->serial_parity_mode = data_cfg_new->serial_parity_mode;
    }

    if ((data_cfg_default->serial_stop_bits != data_cfg_new->serial_stop_bits) && (data_cfg_new->serial_stop_bits)) {
        data_cfg_default->serial_stop_bits = data_cfg_new->serial_stop_bits;
    }
}

static void usart_handler(int vector, void *param)
{
    struct SerialBus *serial_bus = (struct SerialBus *)param;
    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_bus->bus.owner_haldev;
    
    SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
}

#ifdef DEBUG_UART1
static void usart1_handler(int vector, void *param)
{
    struct SerialBus *serial_bus = (struct SerialBus *)param;
    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_bus->bus.owner_haldev;

    // KPrintf("Now we are in uart1 handler %d\n", SERIAL_EVENT_RX_IND);
    
    // NOTE: this cause get_char and filled serial_dev rx_buffer
    SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
}
#endif

static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam *serial_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        SerialCfgParamCheck(serial_cfg, serial_cfg_new);
    }
 
    GPIO_REG(GPIO_IOF_SEL) &= ~IOF0_UART0_MASK;
    GPIO_REG(GPIO_IOF_EN) |= IOF0_UART0_MASK;

    UART0_REG(UART_REG_DIV) = get_cpu_freq() / serial_cfg->data_cfg.serial_baud_rate - 1;
    UART0_REG(UART_REG_TXCTRL) |= UART_TXEN;
    UART0_REG(UART_REG_RXCTRL) |= UART_RXEN;
    UART0_REG(UART_REG_IE) = UART_IP_RXWM;

// #ifdef DEBUG_UART1
    GPIO_REG(GPIO_IOF_SEL) &= ~IOF0_UART1_MASK;
    GPIO_REG(GPIO_IOF_EN) |= IOF0_UART1_MASK;

    UART1_REG(UART_REG_DIV) = get_cpu_freq() / serial_cfg->data_cfg.serial_baud_rate - 1;
    UART1_REG(UART_REG_TXCTRL) |= UART_TXEN;
    UART1_REG(UART_REG_RXCTRL) |= UART_RXEN;
    UART1_REG(UART_REG_IE) = UART_IP_RXWM;
// #endif

    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    return EOK;
}

static uint32 SerialDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int serial_operation_cmd;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = SerialInit(serial_drv, configure_info);
            break;
        case OPE_CFG:
            serial_operation_cmd = *(int *)configure_info->private_data;
            ret = SerialConfigure(serial_drv, serial_operation_cmd);
            break;
        default:
            break;
    }

    return ret;
}

static int SerialPutChar(struct SerialHardwareDevice *serial_dev, char c)
{
#ifdef DEBUG_UART1
    if(strncmp(serial_dev->haldev.dev_name, SERIAL_DEVICE_1_NAME, NAME_NUM_MAX) == 0){
        while (UART1_REG(UART_REG_TXFIFO) & 0x80000000) ;
        UART1_REG(UART_REG_TXFIFO) = c;

        return 0;
    }
#endif
    while (UART0_REG(UART_REG_TXFIFO) & 0x80000000) ;
    UART0_REG(UART_REG_TXFIFO) = c;

    return 0;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
#ifdef DEBUG_UART1
    if(strncmp(serial_dev->haldev.dev_name, SERIAL_DEVICE_1_NAME, NAME_NUM_MAX) == 0){
        int32_t val = UART1_REG(UART_REG_RXFIFO);
        if (val > 0)
            return (uint8_t)val;
        else
            return -1;
    }
#endif    
    int32_t val = UART0_REG(UART_REG_RXFIFO);
    if (val > 0)
        return (uint8_t)val;
    else
        return -1;
}

#ifdef DEBUG_VIRTIO_SERIAL
static uint32 VirtioInit(struct VirtioDriver *virtio_drv, struct BusConfigureInfo *configure_info)
{

    return EOK;
}

static uint32 VirtioConfigure(struct VirtioDriver *virtio_drv, int virtio_operation_cmd)
{
    NULL_PARAM_CHECK(virtio_drv);

    return EOK;
}

static uint32 VirtioDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int virtio_operation_cmd;
    struct VirtioDriver *virtio_drv = (struct VirtioDriver *)drv;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = VirtioInit(virtio_drv, configure_info);
            break;
        case OPE_CFG:
            virtio_operation_cmd = *(int *)configure_info->private_data;
            ret = VirtioConfigure(virtio_drv, virtio_operation_cmd);
            break;
        default:
            break;
    }

    return ret;
}

static void virtio_handler(int vector, void *param)
{
    struct VirtioBus *virtio_bus = (struct VirtioBus *)param;
    struct VirtioHardwareDevice *virtio_dev = (struct VirtioHardwareDevice *)virtio_bus->bus.owner_haldev;

    // KPrintf("Now we are in uart handler %d\n", SERIAL_EVENT_RX_IND);
    
    // NOTE: this cause get_char and filled serial_dev rx_buffer
    // SerialSetIsr(virtio_dev, SERIAL_EVENT_RX_IND);
    // KPrintf("Virtio: Get qemu interrupt and trigger handler funciton!\n");
    switch (virtio_dev->regs->DeviceID)
    {
    case VIRTIO_ID_SERIAL:
        VirtioSerialDeviceGetChar(virtio_dev, SERIAL_EVENT_RX_IND);
        break;
    case VIRTIO_ID_NETWORK:
        VirtioNetDeviceRx(virtio_dev, SERIAL_EVENT_RX_IND);
        break;
    default:
        KPrintf("Virtio Interrupt Handler Not Supported\n");
        break;
    }
}

// TODO: maybe these functions should be in another file
static int VirtioPutChar(struct VirtioHardwareDevice *virtio_dev, char c)
{
    // FIXME: not implemented
    // KPrintf("This is put_char dummy for virtio!\n");
    while (UART1_REG(UART_REG_TXFIFO) & 0x80000000) ;
    UART1_REG(UART_REG_TXFIFO) = c;

    return 0;
}

static int VirtioGetChar(struct VirtioHardwareDevice *virtio_dev)
{
    // KPrintf("This is get_char dummy for virtio!\n");
    int32_t val = UART1_REG(UART_REG_RXFIFO);
    if (val > 0)
        return (uint8_t)val;
    else
        return -1;
}
#endif

static const struct SerialDataCfg data_cfg_init = 
{
    .serial_baud_rate = BAUD_RATE_115200,
    .serial_data_bits = DATA_BITS_8,
    .serial_stop_bits = STOP_BITS_1,
    .serial_parity_mode = PARITY_NONE,
    .serial_bit_order = BIT_ORDER_LSB,
    .serial_invert_mode = NRZ_NORMAL,
    .serial_buffer_size = SERIAL_RB_BUFSZ,
};

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done =
{
    .init = SerialInit,
    .configure = SerialConfigure,
};

static const struct VirtioDrvDone drv_done_virtio =
{
    .init = VirtioInit,
    .configure = VirtioConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done =
{
    .put_char = SerialPutChar,
    .get_char = SerialGetChar,
};

#ifdef DEBUG_VIRTIO_SERIAL
static struct VirtioHwDevDone hwdev_done_virtio =
{
    .put_char = VirtioPutChar,
    .get_char = VirtioGetChar,
};

static int BoardVirtioBusInit(struct VirtioBus *virtio_bus, struct VirtioDriver *virtio_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    /*Init the virtio bus */
    ret = VirtioBusInit(virtio_bus, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart VirtioBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the virtio driver*/
    ret = VirtioDriverInit(virtio_driver, drv_name);
    if (EOK != ret) {
        KPrintf("InitHwUart VirtioDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the virtio driver to the virtio bus*/
    ret = VirtioDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart VirtioDriverAttachToBus error %d\n", ret);
        return ERROR;
    } 

    return ret;
}

/*Attach the virtio device to the virtio bus*/
static int BoardVirtioDevBend(struct VirtioHardwareDevice *virtio_device, void *virtio_param, const char *bus_name, const char *dev_name)
{
    x_err_t ret = EOK;

    ret = VirtioDeviceRegister(virtio_device, virtio_param, dev_name);
    if (EOK != ret) {
        KPrintf("InitHwUart VirtioDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    ret = VirtioDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart VirtioDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    return  ret;
}
#endif

static int BoardSerialBusInit(struct SerialBus *serial_bus, struct SerialDriver *serial_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    /*Init the serial bus */
    ret = SerialBusInit(serial_bus, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the serial driver*/
    ret = SerialDriverInit(serial_driver, drv_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the serial driver to the serial bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverAttachToBus error %d\n", ret);
        return ERROR;
    } 

    return ret;
}

/*Attach the serial device to the serial bus*/
static int BoardSerialDevBend(struct SerialHardwareDevice *serial_device, void *serial_param, const char *bus_name, const char *dev_name)
{
    x_err_t ret = EOK;

    ret = SerialDeviceRegister(serial_device, serial_param, dev_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    ret = SerialDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    return  ret;
}

int InitHwUart(void)
{
    x_err_t ret = EOK;

    static struct SerialBus serial_bus;
    memset(&serial_bus, 0, sizeof(struct SerialBus));

    static struct SerialDriver serial_driver;
    memset(&serial_driver, 0, sizeof(struct SerialDriver));

    static struct SerialHardwareDevice serial_device;
    memset(&serial_device, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg;
    memset(&serial_cfg, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param;
    memset(&serial_dev_param, 0, sizeof(struct SerialDevParam));
    
    serial_driver.drv_done = &drv_done;
    serial_driver.configure = &SerialDrvConfigure;
    serial_device.hwdev_done = &hwdev_done;

    serial_cfg.data_cfg = data_cfg_init;

    serial_cfg.hw_cfg.serial_register_base   = UART0_CTRL_ADDR;
    serial_cfg.hw_cfg.serial_irq_interrupt = INT_UART0_BASE;
    serial_driver.private_data = (void *)&serial_cfg;

    serial_dev_param.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device.haldev.private_data = (void *)&serial_dev_param;

    ret = BoardSerialBusInit(&serial_bus, &serial_driver, SERIAL_BUS_NAME, SERIAL_DRV_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device, (void *)&serial_cfg, SERIAL_BUS_NAME, SERIAL_DEVICE_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }    

    isrManager.done->registerIrq(serial_cfg.hw_cfg.serial_irq_interrupt, usart_handler, &serial_bus);
    isrManager.done->enableIrq(serial_cfg.hw_cfg.serial_irq_interrupt);

#ifdef DEBUG_UART1
static struct SerialBus serial_bus_1;
    memset(&serial_bus_1, 0, sizeof(struct SerialBus));

    static struct SerialDriver serial_driver_1;
    memset(&serial_driver_1, 0, sizeof(struct SerialDriver));

    static struct SerialHardwareDevice serial_device_1;
    memset(&serial_device_1, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_1;
    memset(&serial_cfg_1, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_1;
    memset(&serial_dev_param_1, 0, sizeof(struct SerialDevParam));
    
    serial_driver_1.drv_done = &drv_done;
    serial_driver_1.configure = &SerialDrvConfigure;
    serial_device_1.hwdev_done = &hwdev_done;

    serial_cfg_1.data_cfg = data_cfg_init;

    serial_cfg_1.hw_cfg.serial_register_base   = UART1_CTRL_ADDR;
    serial_cfg_1.hw_cfg.serial_irq_interrupt = INT_UART1_BASE;
    serial_driver_1.private_data = (void *)&serial_cfg_1;

    serial_dev_param_1.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_1.haldev.private_data = (void *)&serial_dev_param_1;

    ret = BoardSerialBusInit(&serial_bus_1, &serial_driver_1, SERIAL_BUS_1_NAME, SERIAL_DRV_1_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void *)&serial_cfg_1, SERIAL_BUS_1_NAME, SERIAL_DEVICE_1_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    } 

    isrManager.done->registerIrq(serial_cfg_1.hw_cfg.serial_irq_interrupt, usart1_handler, &serial_bus_1);
    isrManager.done->enableIrq(serial_cfg_1.hw_cfg.serial_irq_interrupt);
#endif
#ifdef DEBUG_VIRTIO_SERIAL
static struct VirtioBus virtio_bus;
    memset(&virtio_bus, 0, sizeof(struct VirtioBus));

    static struct VirtioDriver virtio_driver;
    memset(&virtio_driver, 0, sizeof(struct VirtioDriver));

    static struct VirtioHardwareDevice virtio_device;
    memset(&virtio_device, 0, sizeof(struct VirtioHardwareDevice));

    VirtioRegs *virtio_regs = (VirtioRegs *)x_malloc(sizeof(VirtioRegs));
    memset((void *)virtio_regs, 0, sizeof(VirtioRegs));

    // static struct VirtioDevParam virtio_dev_param;
    // memset(&virtio_dev_param, 0, sizeof(struct VirtioDevParam));
    
    // TODO: not implemented
    virtio_driver.drv_done = &drv_done_virtio;
    virtio_driver.configure = &VirtioDrvConfigure;
    virtio_device.hwdev_done = &hwdev_done_virtio;

    virtio_regs->MagicValue = VIRTIO_MAGIC;
    virtio_regs->Version = VIRTIO_VERSION;
    virtio_regs->DeviceID = VIRTIO_ID_NETWORK;

    virtio_device.regs = virtio_regs;

    // serial_cfg_1.hw_cfg.serial_register_base   = UART1_CTRL_ADDR;
    // serial_cfg_1.hw_cfg.serial_irq_interrupt = INT_UART1_BASE;
    // virtio_driver.private_data = (void *)&serial_cfg_1;

    // serial_dev_param_1.serial_work_mode = SIGN_OPER_INT_TX;
    // virtio_device.haldev.private_data = (void *)&serial_dev_param_1;

    ret = BoardVirtioBusInit(&virtio_bus, &virtio_driver, VIRTIO_BUS_NAME, VIRTIO_DRV_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardVirtioDevBend(&virtio_device, (void *)virtio_regs, VIRTIO_BUS_NAME, VIRTIO_DEVICE_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    } 

    // TODO: not implemented
    isrManager.done->registerIrq(INT_UART1_BASE, virtio_handler, &virtio_bus);
    isrManager.done->enableIrq(INT_UART1_BASE);
#endif

    return ret;
}
