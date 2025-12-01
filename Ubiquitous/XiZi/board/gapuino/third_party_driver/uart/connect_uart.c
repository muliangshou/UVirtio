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
* @brief support gap8-board uart function and register to bus framework
* @version 1.1
* @author AIIT XUOS Lab
* @date 2021-09-02
*/

#include <xizi.h>
#include <device.h>

#include "connect_uart.h"
#include "hardware_udma.h"
#include "hardware_gpio.h"
#include <board.h>

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

static void UartRxIsr(void *arg)
{
    struct SerialBus *serial_bus = (struct SerialBus *)arg;
    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_bus->bus.owner_haldev;
    
    SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
}

static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    asm volatile("nop");
    NULL_PARAM_CHECK(serial_drv);
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam *serial_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        SerialCfgParamCheck(serial_cfg, serial_cfg_new);
    }

    struct gap8_udma_peripheral *uart_udma = (struct gap8_udma_peripheral *)serial_cfg->hw_cfg.private_data;
    uart_reg_t *uart_reg = (uart_reg_t *)uart_udma->regs;
    uint32_t cfg_reg = 0;

    uint16_t div = CONFIG_CORE_CLOCK_FREQ / serial_cfg->data_cfg.serial_baud_rate;

    gap8_udma_init(uart_udma);

    /* Setup baudrate etc. */
    cfg_reg = UART_SETUP_BIT_LENGTH(serial_cfg->data_cfg.serial_data_bits - 5) |
            UART_SETUP_PARITY_ENA(serial_cfg->data_cfg.serial_parity_mode - 1) |
            UART_SETUP_STOP_BITS(serial_cfg->data_cfg.serial_stop_bits - 1) |
            UART_SETUP_TX_ENA(1) |
            UART_SETUP_RX_ENA(1) |
            UART_SETUP_CLKDIV(div);
    uart_reg->SETUP = cfg_reg;

    gap8_configpin(GAP8_PIN_A7_UART_TX | GAP8_PIN_PULL_UP | GAP8_PIN_SPEED_HIGH);
    gap8_configpin(GAP8_PIN_B6_UART_RX | GAP8_PIN_PULL_UP | GAP8_PIN_SPEED_HIGH);

    return EOK;
}
static char g_uart1rxbuffer[128];
static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct gap8_udma_peripheral *uart_udma = (struct gap8_udma_peripheral *)serial_cfg->hw_cfg.private_data;

    if (OPER_SET_INT == serial_operation_cmd) {
        x_base lock = 0;
        lock = DISABLE_INTERRUPT();

        gap8_udma_rx_setirq(uart_udma, 1);

        gap8_udma_rx_start(uart_udma, g_uart1rxbuffer, 1, 1);

        ENABLE_INTERRUPT(lock);
    } else if (OPER_CLR_INT == serial_operation_cmd) {
        gap8_udma_rx_setirq(uart_udma, 0);

        gap8_udma_deinit(uart_udma);
    }

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
    struct Driver *drv = serial_dev->haldev.owner_bus->owner_driver;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct gap8_udma_peripheral *uart_udma = (struct gap8_udma_peripheral *)serial_cfg->hw_cfg.private_data;
   
    gap8_udma_tx_setirq(uart_udma,1);
    gap8_udma_tx_start(uart_udma, &c, 1, 1);

    for(int i = 999 ; i> 0 ; i--);

    return 0;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    struct Driver *drv = serial_dev->haldev.owner_bus->owner_driver;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct gap8_udma_peripheral *uart_udma = (struct gap8_udma_peripheral *)serial_cfg->hw_cfg.private_data;
    
    uint8_t rx_buf[4] = {0};
    uint8_t ch = g_uart1rxbuffer[0];

    /* Then trigger another reception */
    gap8_udma_rx_setirq(uart_udma, 1);
    gap8_udma_rx_start(uart_udma, g_uart1rxbuffer, 1, 1);

    if (ch == 0)
        return -ERROR;

    memset(g_uart1rxbuffer,0,128);

    return ch;
}

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

static struct gap8_udma_peripheral gap8_udma = 
{
    .regs = (volatile udma_reg_t *)UART,
    .id    = GAP8_UDMA_ID_UART,
    .on_tx = NONE,//UartRxIsr??
    //  .on_tx = UartTxIsr,//UartRxIsr??
    .tx_arg = NONE,
    .on_rx = UartRxIsr,
    .rx_arg = NONE,
    .is_tx_continous = 0,
    .is_rx_continous = 1,
};

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done =
{
    .init = SerialInit,
    .configure = SerialConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done =
{
    .put_char = SerialPutChar,
    .get_char = SerialGetChar,
};

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

    serial_cfg.hw_cfg.private_data = (void *)&gap8_udma;
    serial_driver.private_data = (void *)&serial_cfg;

    serial_dev_param.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device.haldev.private_data = (void *)&serial_dev_param;

    ret = BoardSerialBusInit(&serial_bus, &serial_driver, SERIAL_BUS_NAME_0, SERIAL_DRV_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device, (void *)&serial_cfg, SERIAL_BUS_NAME_0, SERIAL_0_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }  

    gap8_udma.rx_arg = (void *)&serial_bus;

    return ret;
}
