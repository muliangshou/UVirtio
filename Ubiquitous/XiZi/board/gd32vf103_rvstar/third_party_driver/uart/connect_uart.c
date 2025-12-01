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
* @brief supportrvstar-board uart function and register to bus framework
* @version 1.1
* @author AIIT XUOS Lab
* @date 2021-12-03
*/

#include <xizi.h>
#include <device.h>

#include "connect_uart.h"
#include "gd32vf103_usart.h"
#include "gd32vf103.h"
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

static void UartIsr(struct SerialDriver *serial_drv, struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    
    if ((usart_interrupt_flag_get(serial_cfg->hw_cfg.serial_register_base, USART_INT_FLAG_RBNE)
            != RESET)
            && (RESET != usart_flag_get(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_RBNE)))
    {
        SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
        usart_interrupt_flag_clear(serial_cfg->hw_cfg.serial_register_base, USART_INT_FLAG_RBNE);
        usart_flag_clear(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_RBNE);
    }
    else
    {
        if (usart_flag_get(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_CTSF) != RESET)
        {
            usart_flag_clear(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_CTSF);
        }

        if (usart_flag_get(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_LBDF) != RESET)
        {
            usart_flag_clear(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_LBDF);
        }

        if (usart_flag_get(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_TC) != RESET)
        {
            usart_flag_clear(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_TC);
        }
    }

}


static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    // struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    
    usart_deinit(serial_cfg->hw_cfg.serial_register_base);
    usart_baudrate_set(serial_cfg->hw_cfg.serial_register_base, serial_cfg->data_cfg.serial_baud_rate);

    switch (serial_cfg->data_cfg.serial_data_bits)
    {
    case DATA_BITS_8:
        usart_word_length_set(serial_cfg->hw_cfg.serial_register_base, USART_WL_8BIT);
        break;

    case DATA_BITS_9:
        usart_word_length_set(serial_cfg->hw_cfg.serial_register_base, USART_WL_9BIT);
        break;
    default:
        usart_word_length_set(serial_cfg->hw_cfg.serial_register_base, USART_WL_8BIT);
        break;
    }
    
    switch (serial_cfg->data_cfg.serial_stop_bits)
    {
    case STOP_BITS_1:
        usart_stop_bit_set(serial_cfg->hw_cfg.serial_register_base, USART_STB_1BIT);
        break;
    case STOP_BITS_2:
        usart_stop_bit_set(serial_cfg->hw_cfg.serial_register_base, USART_STB_2BIT);
        break;
    default:
        usart_stop_bit_set(serial_cfg->hw_cfg.serial_register_base, USART_STB_1BIT);
        break;
    }
    
    switch (serial_cfg->data_cfg.serial_parity_mode)
    {
    case PARITY_NONE:
        usart_parity_config(serial_cfg->hw_cfg.serial_register_base, USART_PM_NONE);
        break;
    case PARITY_ODD:
        usart_parity_config(serial_cfg->hw_cfg.serial_register_base, USART_PM_ODD);
        break;
    case PARITY_EVEN:
        usart_parity_config(serial_cfg->hw_cfg.serial_register_base, USART_PM_EVEN);
        break;
    default:
        usart_parity_config(serial_cfg->hw_cfg.serial_register_base, USART_PM_NONE);
        break;
    }
    usart_hardware_flow_rts_config(serial_cfg->hw_cfg.serial_register_base, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(serial_cfg->hw_cfg.serial_register_base, USART_CTS_DISABLE);
    usart_receive_config(serial_cfg->hw_cfg.serial_register_base, USART_RECEIVE_ENABLE);
    usart_transmit_config(serial_cfg->hw_cfg.serial_register_base, USART_TRANSMIT_ENABLE);
    usart_enable(serial_cfg->hw_cfg.serial_register_base);

 
    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;

    switch (serial_operation_cmd)
    {
    case OPER_CLR_INT:
        ECLIC_DisableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
        usart_interrupt_disable(serial_cfg->hw_cfg.serial_register_base, USART_INT_RBNE);
        break;
    case OPER_SET_INT:
        ECLIC_EnableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
        /* enable USART0 receive interrupt */
        usart_interrupt_enable(serial_cfg->hw_cfg.serial_register_base, USART_INT_RBNE);
        break;
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
    // struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    usart_data_transmit(serial_cfg->hw_cfg.serial_register_base, (uint8_t) c);
    while (usart_flag_get(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_TBE) == RESET);

    return 0;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    int ch = -1;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;

    if (RESET != usart_flag_get(serial_cfg->hw_cfg.serial_register_base, USART_FLAG_RBNE))
    {
        ch = usart_data_receive(serial_cfg->hw_cfg.serial_register_base) & 0xff;
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
#ifdef BSP_USING_UART4
struct SerialDriver serial_driver_4;
struct SerialHardwareDevice serial_device_4;

void UART4_IRQHandler(int irq_num, void *arg)
{
    UartIsr(&serial_driver_4, &serial_device_4);
}
// DECLARE_HW_IRQ(UART4_IRQn, UART4_IRQHandler, NONE);
#endif

int InitHwUart(void)
{
    x_err_t ret = EOK;

// #ifdef BSP_USING_UART0
//     rcu_periph_clock_enable(RCU_USART0);
// #endif

// #ifdef BSP_USING_UART1
//     rcu_periph_clock_enable(RCU_USART1);
// #endif
// #ifdef BSP_USING_UART2
//     rcu_periph_clock_enable(RCU_USART2);
// #endif
// #ifdef BSP_USING_UART3
//     rcu_periph_clock_enable(RCU_UART3);
// #endif
#ifdef BSP_USING_UART4
    rcu_periph_clock_enable(RCU_UART4);
#endif

#ifdef BSP_USING_UART4
    static struct SerialBus serial_bus;
    memset(&serial_bus, 0, sizeof(struct SerialBus));
    
    memset(&serial_driver_4, 0, sizeof(struct SerialDriver));

    memset(&serial_device_4, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg;
    memset(&serial_cfg, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param;
    memset(&serial_dev_param, 0, sizeof(struct SerialDevParam));
    
    serial_driver_4.drv_done = &drv_done;
    serial_driver_4.configure = &SerialDrvConfigure;
    serial_device_4.hwdev_done = &hwdev_done;

    serial_cfg.data_cfg = data_cfg_init;

    serial_cfg.hw_cfg.serial_register_base   = UART4;
    serial_cfg.hw_cfg.serial_irq_interrupt = UART4_IRQn;

    serial_driver_4.private_data = (void *)&serial_cfg;

    serial_dev_param.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_4.haldev.private_data = (void *)&serial_dev_param;

    ret = BoardSerialBusInit(&serial_bus, &serial_driver_4, SERIAL_BUS_NAME_4, SERIAL_DRV_NAME_4);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_4, (void *)&serial_cfg, SERIAL_BUS_NAME_4, SERIAL_4_DEVICE_NAME_4);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }  
#endif

    return ret;
}
