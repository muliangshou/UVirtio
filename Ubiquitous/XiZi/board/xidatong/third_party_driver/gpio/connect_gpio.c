/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-4-30     misonyo     the first version.
 */

/**
* @file connect_gpio.c
* @brief support gpio function using bus driver framework
* @version 2.0
* @author AIIT XUOS Lab
* @date 2022-03-16
*/

/*************************************************
File name: connect_gpio.c
Description: support gpio configure and register to bus framework
Others: take RT-Thread v4.0.2/bsp/imxrt/libraries/drivers/drv_gpio.c for references
                https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History:
1. Date: 2022-03-16
Author: AIIT XUOS Lab
Modification: add bus driver framework support for gpio
*************************************************/

#include <connect_gpio.h>
#include <fsl_gpio.h>
#include <fsl_iomuxc.h>

struct PinIndex
{
    int index;
    GPIO_Type *gpio;
    uint32_t pin;
};

struct PinIrq
{
    uint8 port_source;
    uint8 pin_source;
    enum IRQn irq_exti_channel;
    uint32 exti_line;
};

struct PinMask
{
    GPIO_Type *gpio;
    uint32 valid_mask;
};

static const IRQn_Type irq_tab[10] =
{
    GPIO1_Combined_0_15_IRQn,
    GPIO1_Combined_16_31_IRQn,
    GPIO2_Combined_0_15_IRQn,
    GPIO2_Combined_16_31_IRQn,
    GPIO3_Combined_0_15_IRQn,
    GPIO3_Combined_16_31_IRQn,
    GPIO4_Combined_0_15_IRQn,
    GPIO4_Combined_16_31_IRQn,
    GPIO5_Combined_0_15_IRQn,
    GPIO5_Combined_16_31_IRQn
};

const struct PinMask pin_mask[] = 
{
    {GPIO1, 0xFFFFFFFF},     /* GPIO1 */
    {GPIO2, 0xFFFFFFFF},     /* GPIO2 */
    {GPIO3, 0x0FFFFFFF},     /* GPIO3,28~31 not supported */
    {GPIO4, 0xFFFFFFFF},     /* GPIO4 */
    {GPIO5, 0x00000007}      /* GPIO5,3~31 not supported */
};

struct PinIrqHdr pin_irq_hdr_tab[] = 
{
/* GPIO1 */
    {-1, 0, NONE, NONE},//1
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},//32
    /* GPIO2 */
    {-1, 0, NONE, NONE},//33
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},//64
    /* GPIO3 */
    {-1, 0, NONE, NONE},//65
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},//96
    /* GPIO4 */
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},//128
    /* GPIO5 */
    {-1, 0, NONE, NONE},//129
    {-1, 0, NONE, NONE},
    {-1, 0, NONE, NONE},
};

#define MUX_BASE            0x401f8014
#define CONFIG_BASE         0x401f8204

#define GPIO5_MUX_BASE      0x400A8000
#define GPIO5_CONFIG_BASE   0x400A8018

const uint8_t reg_offset[] =
{
    42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
    74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,100,101,102,103,104,105,
   112,113,114,115,116,117,118,119,120,121,122,123,106,107,108,109,110,111, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, -1, -1, -1, -1,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
};

static int GetPin(struct PinIndex *pin_index, uint8_t pin)
{
    pin_index->index = pin >> 5;//0:GPIO1 1:GPIO2 2:GPIO3 3:GPIO4 4:GPIO5 
    pin_index->pin = pin & 31;//each GPIOx support 32 io
    
    if ((pin_index->index > 4) || ((pin_mask[pin_index->index].valid_mask & (1 << pin_index->pin)) == 0)) {
        KPrintf("GetPin unsupport pin index %u pin %u\n", pin_index->index, pin_index->pin);
        return -1;
    }

    pin_index->gpio = pin_mask[pin_index->index].gpio;
    return 0;
}

static int32 GpioConfigMode(int mode, struct PinIndex *pin_index, int32 pin)
{
    gpio_pin_config_t gpio_config;
    uint32_t config_value = 0;
    NULL_PARAM_CHECK(pin_index);

    gpio_config.outputLogic = 0;
    gpio_config.interruptMode = kGPIO_NoIntmode;

    switch (mode)
    {
        case GPIO_CFG_OUTPUT:
            gpio_config.direction = kGPIO_DigitalOutput;
            config_value = 0x0030U;    /* Drive Strength R0/6 */
            break;
        case GPIO_CFG_INPUT:
            gpio_config.direction = kGPIO_DigitalInput;
            config_value = 0x0830U;    /* Open Drain Enable */
            break;
        case GPIO_CFG_INPUT_PULLUP:
            gpio_config.direction = kGPIO_DigitalInput;
            config_value = 0xB030U;    /* 100K Ohm Pull Up */
            break;
        case GPIO_CFG_INPUT_PULLDOWN:
            gpio_config.direction = kGPIO_DigitalInput;
            config_value = 0x3030U;    /* 100K Ohm Pull Down */
            break;
        case GPIO_CFG_OUTPUT_OD:
            gpio_config.direction = kGPIO_DigitalOutput;
            config_value = 0x0830U;    /* Open Drain Enable */
            break;
        default:
            break;
    }

    if (pin_mask[pin_index->index].gpio != GPIO5) {
        CLOCK_EnableClock(kCLOCK_Iomuxc);
        IOMUXC_SetPinMux(MUX_BASE + reg_offset[pin] * 4, 0x5U, 0, 0, CONFIG_BASE + reg_offset[pin] * 4, 1);
        IOMUXC_SetPinConfig(MUX_BASE + reg_offset[pin] * 4, 0x5U, 0, 0, CONFIG_BASE + reg_offset[pin] * 4, config_value);
    } else {
        CLOCK_EnableClock(kCLOCK_IomuxcSnvs);
        IOMUXC_SetPinMux(GPIO5_MUX_BASE + pin_index->pin * 4, 0x5U, 0, 0, GPIO5_CONFIG_BASE + pin_index->pin * 4, 1);
        IOMUXC_SetPinConfig(GPIO5_MUX_BASE + pin_index->pin * 4, 0x5U, 0, 0, GPIO5_CONFIG_BASE + pin_index->pin * 4, config_value);
    }

    GPIO_PinInit(pin_index->gpio, pin_index->pin, &gpio_config);
    return EOK;
}

static int32 GpioIrqRegister(int32 pin, int32 mode, void (*hdr)(void *args), void *args)
{
    struct PinIndex pin_index;

    if (GetPin(&pin_index, pin) < 0) {
        return ERROR;
    }

    x_base level = CriticalAreaLock();
    if (pin_irq_hdr_tab[pin].pin == pin   &&
        pin_irq_hdr_tab[pin].hdr == hdr   &&
        pin_irq_hdr_tab[pin].mode == mode &&
        pin_irq_hdr_tab[pin].args == args
    )
    {
        CriticalAreaUnLock(level);
        return EOK;
    }

    if (pin_irq_hdr_tab[pin].pin != -1) {
        CriticalAreaUnLock(level);
        return -EDEV_BUSY;
    }
    pin_irq_hdr_tab[pin].pin = pin;
    pin_irq_hdr_tab[pin].hdr = hdr;
    pin_irq_hdr_tab[pin].mode = mode;
    pin_irq_hdr_tab[pin].args = args;
    CriticalAreaUnLock(level);

    return EOK;
}

static uint32 GpioIrqFree(int32 pin)
{
    struct PinIndex pin_index;

    if (GetPin(&pin_index, pin) < 0) {
        return ERROR;
    }

    x_base level = CriticalAreaLock();
    if (pin_irq_hdr_tab[pin].pin == -1){
        CriticalAreaUnLock(level);
        return EOK;
    }
    pin_irq_hdr_tab[pin].pin  = -1;
    pin_irq_hdr_tab[pin].hdr  = NONE;
    pin_irq_hdr_tab[pin].mode = 0;
    pin_irq_hdr_tab[pin].args = NONE;
    CriticalAreaUnLock(level);

    return EOK;
}

static int32 GpioIrqEnable(x_base pin)
{
    uint8_t irq_index;
    gpio_interrupt_mode_t gpio_int_mode;
    struct PinIndex pin_index;

    if (GetPin(&pin_index, pin) < 0) {
        return ERROR;
    }

    x_base level = CriticalAreaLock();
    if (pin_irq_hdr_tab[pin].pin == -1) {
        CriticalAreaUnLock(level);
        return -ENONESYS;
    }

    switch (pin_irq_hdr_tab[pin].mode)
    {
        case GPIO_IRQ_EDGE_RISING:
            gpio_int_mode = kGPIO_IntRisingEdge;
            break;
        case GPIO_IRQ_EDGE_FALLING:
            gpio_int_mode = kGPIO_IntFallingEdge;
            break;
        case GPIO_IRQ_EDGE_BOTH:
            gpio_int_mode = kGPIO_IntRisingOrFallingEdge;
            break;
        case GPIO_IRQ_LEVEL_HIGH:
            gpio_int_mode = kGPIO_IntHighLevel;
            break;
        case GPIO_IRQ_LEVEL_LOW:
            gpio_int_mode = kGPIO_IntLowLevel;
            break;
        default:
            gpio_int_mode = kGPIO_IntRisingEdge;
            break;
    }

    irq_index = (pin_index.index << 1) + (pin_index.pin >> 4);
    GPIO_PinSetInterruptConfig(pin_index.gpio, pin_index.pin, gpio_int_mode);
    GPIO_PortEnableInterrupts(pin_index.gpio, 1U << pin_index.pin);
    NVIC_SetPriority(irq_tab[irq_index], NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    EnableIRQ(irq_tab[irq_index]);
    CriticalAreaUnLock(level);
    return EOK;
}

static int32 GpioIrqDisable(x_base pin)
{
    struct PinIndex pin_index;

    if (GetPin(&pin_index, pin) < 0) {
        return ERROR;
    }

    GPIO_PortDisableInterrupts(pin_index.gpio, 1U << pin_index.pin);
    return EOK;
}

static uint32 Imxrt1052PinConfigure(struct PinParam *param)
{
    NULL_PARAM_CHECK(param);
    int ret = EOK;

    struct PinIndex pin_index;

    if (GetPin(&pin_index, param->pin) < 0) {
        return ERROR;
    }

    switch(param->cmd)
    {
        case GPIO_CONFIG_MODE:
            GpioConfigMode(param->mode, &pin_index, param->pin);
            break;
        case GPIO_IRQ_REGISTER:
            ret = GpioIrqRegister(param->pin, param->irq_set.irq_mode, param->irq_set.hdr, param->irq_set.args);
            break;
        case GPIO_IRQ_FREE:
            ret = GpioIrqFree(param->pin);
            break;
        case GPIO_IRQ_ENABLE:
            ret = GpioIrqEnable(param->pin);
            break;
        case GPIO_IRQ_DISABLE:
            ret = GpioIrqDisable(param->pin);
            break;
        default:
            ret = -EINVALED;
            break;
    }

    return ret;
}

static uint32 Imxrt1052PinInit(void)
{
    static x_bool pin_init_flag = RET_FALSE;

    if (!pin_init_flag) {
        pin_init_flag = RET_TRUE;
    }

    return EOK;
}

static uint32 Imxrt1052GpioDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    struct PinParam *param;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = Imxrt1052PinInit();
            break;
        case OPE_CFG:
            param = (struct PinParam *)configure_info->private_data;
            ret = Imxrt1052PinConfigure(param);
            break;
        default:
            break;
    }

    return ret;
}

uint32 Imxrt1052PinWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(write_param);
    struct PinStat *pin_stat = (struct PinStat *)write_param->buffer;
    struct PinIndex pin_index;
    
    if (GetPin(&pin_index, pin_stat->pin) < 0) {
        return ERROR;
    }

    if (GPIO_LOW == pin_stat->val) {
        GPIO_PinWrite(pin_index.gpio, pin_index.pin, 0);
    } else {
        GPIO_PinWrite(pin_index.gpio, pin_index.pin, 1);
    }
    return EOK;
}

uint32 Imxrt1052PinRead(void *dev, struct BusBlockReadParam *read_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(read_param);
    struct PinStat *pin_stat = (struct PinStat *)read_param->buffer;
    struct PinIndex pin_index;
    
    if (GetPin(&pin_index, pin_stat->pin) < 0) {
        return ERROR;
    }

    if(GPIO_LOW == GPIO_PinRead(pin_index.gpio, pin_index.pin)) {
        pin_stat->val = GPIO_LOW;
    } else {
        pin_stat->val = GPIO_HIGH;
    }
    return pin_stat->val;
}

static const struct PinDevDone dev_done =
{
    .open  = NONE,
    .close = NONE,
    .write = Imxrt1052PinWrite,
    .read  = Imxrt1052PinRead,
};

int Imxrt1052HwGpioInit(void)
{
    x_err_t ret = EOK;

    static struct PinBus pin;

    ret = PinBusInit(&pin, PIN_BUS_NAME);
    if (ret != EOK) {
        KPrintf("gpio bus init error %d\n", ret);
        return ERROR;
    }

    static struct PinDriver drv;
    drv.configure = Imxrt1052GpioDrvConfigure;

    ret = PinDriverInit(&drv, PIN_DRIVER_NAME, NONE);
    if (ret != EOK) {
        KPrintf("pin driver init error %d\n", ret);
        return ERROR;
    }
    ret = PinDriverAttachToBus(PIN_DRIVER_NAME, PIN_BUS_NAME);
    if (ret != EOK) {
        KPrintf("pin driver attach error %d\n", ret);
        return ERROR;
    }

    static struct PinHardwareDevice dev;
    dev.dev_done = &dev_done;

    ret = PinDeviceRegister(&dev, NONE, PIN_DEVICE_NAME);
    if (ret != EOK) {
        KPrintf("pin device register error %d\n", ret);
        return ERROR;
    }
    ret = PinDeviceAttachToBus(PIN_DEVICE_NAME, PIN_BUS_NAME);
    if (ret != EOK) {
        KPrintf("pin device register error %d\n", ret);
        return ERROR;
    }

    return ret;
}

static __inline void PinIrqHdr(uint32_t index_offset, uint8_t pin_start, GPIO_Type *gpio)
{
    int i;
    uint32_t isr_status, pin;
    struct PinIndex pin_index;

    isr_status = GPIO_PortGetInterruptFlags(gpio) & gpio->IMR;

    for (i = pin_start; i <= pin_start + 15 ; i ++) {

        if (GetPin(&pin_index, i + index_offset) < 0) {
            continue;
        }

        if (isr_status & (1 << i)) {
            GPIO_PortClearInterruptFlags(gpio, (1 << i));

            __DSB();

            pin = index_offset + i;
            if (pin_irq_hdr_tab[pin].hdr) {
                pin_irq_hdr_tab[pin].hdr(pin_irq_hdr_tab[pin].args);
            }
        }
    }
}

void GPIO1_0_15_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(0, 0, GPIO1);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO1_Combined_0_15_IRQn, GPIO1_0_15_IRQHandler, NONE);

void GPIO1_16_31_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(0, 15, GPIO1);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO1_Combined_16_31_IRQn, GPIO1_16_31_IRQHandler, NONE);

void GPIO2_0_15_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(32, 0, GPIO2);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO2_Combined_0_15_IRQn, GPIO2_0_15_IRQHandler, NONE);

void GPIO2_16_31_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(32, 15, GPIO2);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO2_Combined_16_31_IRQn, GPIO2_16_31_IRQHandler, NONE);

void GPIO3_0_15_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(64, 0, GPIO3);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO3_Combined_0_15_IRQn, GPIO3_0_15_IRQHandler, NONE);

void GPIO3_16_31_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(64, 15, GPIO3);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO3_Combined_16_31_IRQn, GPIO3_16_31_IRQHandler, NONE);

void GPIO4_0_15_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(96, 0, GPIO4);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO4_Combined_0_15_IRQn, GPIO4_0_15_IRQHandler, NONE);

void GPIO4_16_31_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(96, 15, GPIO4);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO4_Combined_16_31_IRQn, GPIO4_16_31_IRQHandler, NONE);

void GPIO5_0_15_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(128, 0, GPIO5);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO5_Combined_0_15_IRQn, GPIO5_0_15_IRQHandler, NONE);

void GPIO5_16_31_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();

    PinIrqHdr(128, 15, GPIO5);

    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(GPIO5_Combined_16_31_IRQn, GPIO5_16_31_IRQHandler, NONE);

#ifdef GPIO_LED_TEST
static void GpioLedDelay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 8000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void GpioLedTest(void)
{
	BusType pin;
    struct BusConfigureInfo configure_info;
	struct BusBlockWriteParam write_param;

    int ret = 0;
    bool pinSet = 1;

    pin = BusFind(PIN_BUS_NAME);
    if (!pin) {
        KPrintf("find %s failed!\n", PIN_BUS_NAME);
        return;
    }
    pin->owner_driver = BusFindDriver(pin, PIN_DRIVER_NAME);
    pin->owner_haldev = BusFindDevice(pin, PIN_DEVICE_NAME);

	configure_info.configure_cmd = OPE_INT;
    ret = BusDrvConfigure(pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("initialize %s failed!\n", PIN_BUS_NAME);
        return;
    }
    
    struct PinParam led_gpio_param;
    struct PinStat led_gpio_stat;

    /* config led pin as output*/
    led_gpio_param.cmd = GPIO_CONFIG_MODE;
    led_gpio_param.pin = IMXRT_GET_PIN(1, 9);
    led_gpio_param.mode = GPIO_CFG_OUTPUT_OD;

	configure_info.configure_cmd = OPE_CFG;
	configure_info.private_data = (void *)&led_gpio_param;
    ret = BusDrvConfigure(pin->owner_driver, &configure_info);
    if (ret != EOK) {
        KPrintf("config pin %d failed!\n", IMXRT_GET_PIN(1, 9));
        return;
    }

    while (1) {

        GpioLedDelay();

        if (pinSet) {
            /* set led pin as high*/
            led_gpio_stat.pin = IMXRT_GET_PIN(1, 9);
            led_gpio_stat.val = GPIO_HIGH;
            write_param.buffer = (void *)&led_gpio_stat;
            BusDevWriteData(pin->owner_haldev, &write_param);
            pinSet = 0;
        } else {
            /* set led pin as low*/
            led_gpio_stat.pin = IMXRT_GET_PIN(1, 9);
            led_gpio_stat.val = GPIO_LOW;
            write_param.buffer = (void *)&led_gpio_stat;
            BusDevWriteData(pin->owner_haldev, &write_param);
            pinSet = 1;
        }

    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                                                GpioLedTest, GpioLedTest, GpioLedTest GPIO1 IO09 LED);
#endif
