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
* @file connect_uart.c
* @brief support nuvoton_m2354 board uart function and register to bus framework
* @version 1.1
* @author AIIT XUOS Lab
* @date 2022-02-24
*/

#include <board.h>
#include <connect_uart.h>

struct M2354UartHwCfg
{
    UART_T *uart_handle;
    uint32 uart_rst_index;
    IRQn_Type irq_type;
};

#ifdef BSP_USING_UART0
static struct SerialBus serial_bus_0;
static struct SerialDriver serial_driver_0;
static struct SerialHardwareDevice serial_device_0;
#endif

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

static void UartHandler(struct SerialBus *serial_bus, struct SerialDriver *serial_drv)
{
    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_bus->bus.owner_haldev;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data; 
    struct M2354UartHwCfg *serial_hw_cfg = (struct M2354UartHwCfg *)serial_cfg->hw_cfg.private_data;

    /* Get interrupt event */
    uint32 u32IntSts = serial_hw_cfg->uart_handle->INTSTS;
    uint32 u32FIFOSts = serial_hw_cfg->uart_handle->FIFOSTS;

    /* Handle RX event */
    if (u32IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
    }
    serial_hw_cfg->uart_handle->INTSTS = u32IntSts;
    serial_hw_cfg->uart_handle->FIFOSTS = u32FIFOSts;
}

#ifdef BSP_USING_UART0
void UART0_IRQHandler(int irqn, void *arg)
{
    x_base lock = 0;

    lock = DISABLE_INTERRUPT();

    UartHandler(&serial_bus_0, &serial_driver_0);

    ENABLE_INTERRUPT(lock);
}
//DECLARE_HW_IRQ(UART0_IRQ_NUM, UART0_IRQHandler, NONE);
#endif

static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);

    uint32 uart_word_len = 0;
    uint32 uart_stop_bit = 0;
    uint32 uart_parity = 0;

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct M2354UartHwCfg *serial_hw_cfg = (struct M2354UartHwCfg *)serial_cfg->hw_cfg.private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam *serial_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        SerialCfgParamCheck(serial_cfg, serial_cfg_new);
    }

    switch (serial_cfg->data_cfg.serial_data_bits)
    {
    case DATA_BITS_5:
        uart_word_len = UART_WORD_LEN_5;
        break;
    case DATA_BITS_6:
        uart_word_len = UART_WORD_LEN_6;
        break;
    case DATA_BITS_7:
        uart_word_len = UART_WORD_LEN_7;
        break;
    case DATA_BITS_8:
        uart_word_len = UART_WORD_LEN_8;
        break;
    default:
        KPrintf("SerialInit Unsupported serial_data_bits %u\n", serial_cfg->data_cfg.serial_data_bits);
        return ERROR;
    }

    switch (serial_cfg->data_cfg.serial_stop_bits)
    {
    case STOP_BITS_1:
        uart_stop_bit = UART_STOP_BIT_1;
        break;
    case STOP_BITS_2:
        uart_stop_bit = UART_STOP_BIT_2;
        break;
    default:
        KPrintf("SerialInit Unsupported serial_stop_bits %u\n", serial_cfg->data_cfg.serial_stop_bits);
        return ERROR;
    }

    switch (serial_cfg->data_cfg.serial_parity_mode)
    {
    case PARITY_NONE:
        uart_parity = UART_PARITY_NONE;
        break;
    case PARITY_ODD:
        uart_parity = UART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        uart_parity = UART_PARITY_EVEN;
        break;
    default:
        KPrintf("SerialInit Unsupported serial_parity_mode %u\n", serial_cfg->data_cfg.serial_parity_mode);
        return ERROR;
    }

    /* Reset this module */
    SYS_ResetModule(serial_hw_cfg->uart_rst_index);

    /* Open Uart and set UART Baudrate */
    UART_Open(serial_hw_cfg->uart_handle, serial_cfg->data_cfg.serial_baud_rate);

    /* Set line configuration. */
    UART_SetLineConfig(serial_hw_cfg->uart_handle, 0, uart_word_len, uart_parity, uart_stop_bit);

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(serial_hw_cfg->irq_type);

    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct M2354UartHwCfg *serial_hw_cfg = (struct M2354UartHwCfg *)serial_cfg->hw_cfg.private_data;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

    if (OPER_CLR_INT == serial_operation_cmd) {
        if (SIGN_OPER_INT_RX & serial_dev_param->serial_work_mode) {
            UART_DISABLE_INT(serial_hw_cfg->uart_handle, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk);
        }
	} else if (OPER_SET_INT == serial_operation_cmd) {
        UART_ENABLE_INT(serial_hw_cfg->uart_handle, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk);
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
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct M2354UartHwCfg *serial_hw_cfg = (struct M2354UartHwCfg *)serial_cfg->hw_cfg.private_data;

    /* Waiting if TX-FIFO is full. */
    while (UART_IS_TX_FULL(serial_hw_cfg->uart_handle));

    /* Put char into TX-FIFO */
    UART_WRITE(serial_hw_cfg->uart_handle, c);

    return EOK;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct M2354UartHwCfg *serial_hw_cfg = (struct M2354UartHwCfg *)serial_cfg->hw_cfg.private_data;

    /* Return failure if RX-FIFO is empty. */
    if (UART_GET_RX_EMPTY(serial_hw_cfg->uart_handle)) {
        return -1;
    }

    /* Get char from RX-FIFO */
    return UART_READ(serial_hw_cfg->uart_handle);
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
        KPrintf("M2354HwUartInit SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the serial driver*/
    ret = SerialDriverInit(serial_driver, drv_name);
    if (EOK != ret) {
        KPrintf("M2354HwUartInit SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the serial driver to the serial bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("M2354HwUartInit SerialDriverAttachToBus error %d\n", ret);
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
        KPrintf("M2354HwUartInit SerialDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    ret = SerialDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("M2354HwUartInit SerialDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    return  ret;
}

int M2354HwUartInit(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_UART0
    memset(&serial_bus_0, 0, sizeof(struct SerialBus));
    memset(&serial_driver_0, 0, sizeof(struct SerialDriver));
    memset(&serial_device_0, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_0;
    memset(&serial_cfg_0, 0, sizeof(struct SerialCfgParam));

    static struct M2354UartHwCfg serial_hw_cfg_0;
    memset(&serial_hw_cfg_0, 0, sizeof(struct M2354UartHwCfg));

    static struct SerialDevParam serial_dev_param_0;
    memset(&serial_dev_param_0, 0, sizeof(struct SerialDevParam));
    
    serial_driver_0.drv_done = &drv_done;
    serial_driver_0.configure = SerialDrvConfigure;
    serial_device_0.hwdev_done = &hwdev_done;

    serial_cfg_0.data_cfg = data_cfg_init;

    serial_cfg_0.hw_cfg.private_data = (void *)&serial_hw_cfg_0;
    serial_hw_cfg_0.uart_handle = UART0;
    serial_hw_cfg_0.uart_rst_index = UART0_RST;
    serial_hw_cfg_0.irq_type = UART0_IRQn;
    serial_driver_0.private_data = (void *)&serial_cfg_0;

    serial_dev_param_0.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_0.haldev.private_data = (void *)&serial_dev_param_0;

    ret = BoardSerialBusInit(&serial_bus_0, &serial_driver_0, SERIAL_BUS_NAME_0, SERIAL_DRV_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_0, (void *)&serial_cfg_0, SERIAL_BUS_NAME_0, SERIAL_0_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }    
#endif

    return ret;
}














