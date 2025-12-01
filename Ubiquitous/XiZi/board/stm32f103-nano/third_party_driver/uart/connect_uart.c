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
* @brief support stm32f103_nano board uart function and register to bus framework
* @version 1.1
* @author AIIT XUOS Lab
* @date 2021-11-25
*/

#include <board.h>
#include <connect_uart.h>

#ifdef BSP_USING_UART1
static struct SerialBus serial_bus_1;
static struct SerialDriver serial_driver_1;
static struct SerialHardwareDevice serial_device_1;
#endif
#ifdef BSP_USING_UART2
static struct SerialBus serial_bus_2;
static struct SerialDriver serial_driver_2;
static struct SerialHardwareDevice serial_device_2;
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
    struct Stm32UartHwCfg *serial_hw_cfg = (struct Stm32UartHwCfg *)serial_cfg->hw_cfg.private_data;

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&(serial_hw_cfg->uart_handle), UART_FLAG_RXNE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&(serial_hw_cfg->uart_handle), UART_IT_RXNE) != RESET))
    {
        SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
    }
    else
    {
        if (__HAL_UART_GET_FLAG(&(serial_hw_cfg->uart_handle), UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(&serial_hw_cfg->uart_handle);
        }
        if (__HAL_UART_GET_FLAG(&(serial_hw_cfg->uart_handle), UART_FLAG_NE) != RESET)
        {
            __HAL_UART_CLEAR_NEFLAG(&serial_hw_cfg->uart_handle);
        }
        if (__HAL_UART_GET_FLAG(&(serial_hw_cfg->uart_handle), UART_FLAG_FE) != RESET)
        {
            __HAL_UART_CLEAR_FEFLAG(&serial_hw_cfg->uart_handle);
        }
        if (__HAL_UART_GET_FLAG(&(serial_hw_cfg->uart_handle), UART_FLAG_PE) != RESET)
        {
            __HAL_UART_CLEAR_PEFLAG(&serial_hw_cfg->uart_handle);
        }
    }
}

#ifdef BSP_USING_UART1
void UartIsr1(int vector, void *param)
{
	/* get serial bus 1 */
	UartHandler(&serial_bus_1, &serial_driver_1);
}
#endif

#ifdef BSP_USING_UART2
void UartIsr2(int vector, void *param)
{
	/* get serial bus 2 */
	UartHandler(&serial_bus_2, &serial_driver_2);
}
#endif

static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct Stm32UartHwCfg *serial_hw_cfg = (struct Stm32UartHwCfg *)serial_cfg->hw_cfg.private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam *serial_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        SerialCfgParamCheck(serial_cfg, serial_cfg_new);
    }

    serial_hw_cfg->uart_handle.Instance          = serial_hw_cfg->uart_device;
    serial_hw_cfg->uart_handle.Init.BaudRate     = serial_cfg->data_cfg.serial_baud_rate;
    serial_hw_cfg->uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    serial_hw_cfg->uart_handle.Init.Mode         = UART_MODE_TX_RX;
    serial_hw_cfg->uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;

    switch (serial_cfg->data_cfg.serial_data_bits)
    {
    case DATA_BITS_8:
        if (serial_cfg->data_cfg.serial_parity_mode == PARITY_ODD || serial_cfg->data_cfg.serial_parity_mode == PARITY_EVEN)
            serial_hw_cfg->uart_handle.Init.WordLength = UART_WORDLENGTH_9B;
        else
            serial_hw_cfg->uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case DATA_BITS_9:
        serial_hw_cfg->uart_handle.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        serial_hw_cfg->uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    }

    switch (serial_cfg->data_cfg.serial_stop_bits)
    {
    case STOP_BITS_1:
        serial_hw_cfg->uart_handle.Init.StopBits   = UART_STOPBITS_1;
        break;
    case STOP_BITS_2:
        serial_hw_cfg->uart_handle.Init.StopBits   = UART_STOPBITS_2;
        break;
    default:
        serial_hw_cfg->uart_handle.Init.StopBits   = UART_STOPBITS_1;
        break;
    }

    switch (serial_cfg->data_cfg.serial_parity_mode)
    {
    case PARITY_NONE:
        serial_hw_cfg->uart_handle.Init.Parity     = UART_PARITY_NONE;
        break;
    case PARITY_ODD:
        serial_hw_cfg->uart_handle.Init.Parity     = UART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        serial_hw_cfg->uart_handle.Init.Parity     = UART_PARITY_EVEN;
        break;
    default:
        serial_hw_cfg->uart_handle.Init.Parity     = UART_PARITY_NONE;
        break;
    }

    if (HAL_UART_Init(&serial_hw_cfg->uart_handle) != HAL_OK)
    {
        return ERROR;
    }

    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct Stm32UartHwCfg *serial_hw_cfg = (struct Stm32UartHwCfg *)serial_cfg->hw_cfg.private_data;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

    if (OPER_CLR_INT == serial_operation_cmd) {
        if (SIGN_OPER_INT_RX & serial_dev_param->serial_work_mode) {
            /* disable rx irq */
            NVIC_DisableIRQ(serial_hw_cfg->irq_type);
            /* disable interrupt */
            __HAL_UART_DISABLE_IT(&(serial_hw_cfg->uart_handle), UART_IT_RXNE);
        }
	} else if (OPER_SET_INT == serial_operation_cmd) {
        /* enable rx irq */
        HAL_NVIC_SetPriority(serial_hw_cfg->irq_type, 1, 0);
        HAL_NVIC_EnableIRQ(serial_hw_cfg->irq_type);
        /* enable interrupt */
        __HAL_UART_ENABLE_IT(&(serial_hw_cfg->uart_handle), UART_IT_RXNE);
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
    struct Stm32UartHwCfg *serial_hw_cfg = (struct Stm32UartHwCfg *)serial_cfg->hw_cfg.private_data;

    UART_INSTANCE_CLEAR_FUNCTION(&(serial_hw_cfg->uart_handle), UART_FLAG_TC);

    serial_hw_cfg->uart_handle.Instance->DR = c;
    while (__HAL_UART_GET_FLAG(&(serial_hw_cfg->uart_handle), UART_FLAG_TC) == RESET);

    return EOK;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct Stm32UartHwCfg *serial_hw_cfg = (struct Stm32UartHwCfg *)serial_cfg->hw_cfg.private_data;

    int ch = -1;
    if (__HAL_UART_GET_FLAG(&(serial_hw_cfg->uart_handle), UART_FLAG_RXNE) != RESET)
    {
        ch = serial_hw_cfg->uart_handle.Instance->DR & 0xff;
    }
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

#ifdef BSP_USING_UART1
    memset(&serial_bus_1, 0, sizeof(struct SerialBus));
    memset(&serial_driver_1, 0, sizeof(struct SerialDriver));
    memset(&serial_device_1, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_1;
    memset(&serial_cfg_1, 0, sizeof(struct SerialCfgParam));

    static struct Stm32UartHwCfg serial_hw_cfg_1;
    memset(&serial_hw_cfg_1, 0, sizeof(struct Stm32UartHwCfg));

    static struct SerialDevParam serial_dev_param_1;
    memset(&serial_dev_param_1, 0, sizeof(struct SerialDevParam));
    
    serial_driver_1.drv_done = &drv_done;
    serial_driver_1.configure = &SerialDrvConfigure;
    serial_device_1.hwdev_done = &hwdev_done;

    serial_cfg_1.data_cfg = data_cfg_init;

    serial_cfg_1.hw_cfg.private_data = (void *)&serial_hw_cfg_1;
    serial_hw_cfg_1.uart_device = USART1;
    serial_hw_cfg_1.irq_type = USART1_IRQn;
    serial_driver_1.private_data = (void *)&serial_cfg_1;

    serial_dev_param_1.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_1.haldev.private_data = (void *)&serial_dev_param_1;

    ret = BoardSerialBusInit(&serial_bus_1, &serial_driver_1, SERIAL_BUS_NAME_1, SERIAL_DRV_NAME_1);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void *)&serial_cfg_1, SERIAL_BUS_NAME_1, SERIAL_1_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }    
#endif

#ifdef BSP_USING_UART2
    memset(&serial_bus_2, 0, sizeof(struct SerialBus));
    memset(&serial_driver_2, 0, sizeof(struct SerialDriver));
    memset(&serial_device_2, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_2;
    memset(&serial_cfg_2, 0, sizeof(struct SerialCfgParam));

    static struct Stm32UartHwCfg serial_hw_cfg_2;
    memset(&serial_hw_cfg_2, 0, sizeof(struct Stm32UartHwCfg));

    static struct SerialDevParam serial_dev_param_2;
    memset(&serial_dev_param_2, 0, sizeof(struct SerialDevParam));
    
    serial_driver_2.drv_done = &drv_done;
    serial_driver_2.configure = &SerialDrvConfigure;
    serial_device_2.hwdev_done = &hwdev_done;

    serial_cfg_2.data_cfg = data_cfg_init;

    serial_cfg_2.hw_cfg.private_data = (void *)&serial_hw_cfg_2;
    serial_hw_cfg_2.uart_device = USART2;
    serial_hw_cfg_2.irq_type = USART2_IRQn;
    serial_driver_2.private_data = (void *)&serial_cfg_2;

    serial_dev_param_2.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_2.haldev.private_data = (void *)&serial_dev_param_2;

    ret = BoardSerialBusInit(&serial_bus_2, &serial_driver_2, SERIAL_BUS_NAME_2, SERIAL_DRV_NAME_2);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_2, (void *)&serial_cfg_2, SERIAL_BUS_NAME_2, SERIAL_2_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }    
#endif

    return ret;
}
