/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
* @file connect_usb.c
* @brief support usb host function using bus driver framework on xidatong board
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-02-09
*/

/*************************************************
File name: connect_usb.c
Description: support imxrt1052-board usb host configure and sdio bus register function
Others: take SDK_2.6.1_MIMXRT1052xxxxB/boards/evkbimxrt1050/usb_examples/usb_host_msd_command for references
History: 
1. Date: 2022-02-09
Author: AIIT XUOS Lab
Modification: 
1. support imxrt1052-board usb host configure, write and read
2. support imxrt1052-board usb host bus device and driver register
*************************************************/
#include <board.h>
#include <connect_usb.h>

#define BSP_USING_NXP_USBH
#ifdef BSP_USING_NXP_USBH

/*! @brief USB host msd command instance global variable */
extern usb_host_msd_command_instance_t g_MsdCommandInstance;
usb_host_handle g_HostHandle;

extern usb_status_t USB_HostMsdReadApi(usb_host_msd_command_instance_t *msdCommandInstance, uint8_t *buffer, uint32_t pos, uint32_t block_size, uint32_t block_num);
extern usb_status_t USB_HostMsdWriteApi(usb_host_msd_command_instance_t *msdCommandInstance, const uint8_t *buffer, uint32_t pos, uint32_t block_size, uint32_t block_num);

//USB HOST ISR
void UsbOtg2IrqHandler(int irqn, void *arg)
{

    DisableIRQ(USB2_IRQn);

    USB_HostEhciIsrFunction(g_HostHandle);
    EnableIRQ(USB2_IRQn);

}
DECLARE_HW_IRQ(USB2_IRQn, UsbOtg2IrqHandler, NONE);

void UsbHostClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    if (CONTROLLER_ID == kUSB_ControllerEhci0) {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    } else {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

void UsbHostIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber                = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
/* USB_HOST_CONFIG_EHCI */

/* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}

void UsbHostTaskFn(void *param)
{
    USB_HostEhciTaskFunction(param);
}

/*!
 * @brief USB isr function.
 */
static usb_status_t UsbHostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    switch (eventCode)
    {
        case kUSB_HostEventAttach:
            KPrintf("usb device attached\n");
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventNotSupported:
            KPrintf("device not supported.\r\n");
            break;

        case kUSB_HostEventEnumerationDone:
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventDetach:
            KPrintf("usb device detached\n");
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        default:
            break;
    }
    return status;
}

static void UsbHostApplicationInit(void)
{
    usb_status_t status = kStatus_USB_Success;

    UsbHostClockInit();

    status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, UsbHostEvent);
    if (status != kStatus_USB_Success) {
        KPrintf("host init error\r\n");
        return;
    }
    UsbHostIsrEnable();

    KPrintf("host init done\r\n");
}

#if defined(FS_VFS)
void UsbMountFileSystem()
{
    if (MountFilesystem(USB_BUS_NAME, USB_DEVICE_NAME, USB_DRIVER_NAME, FSTYPE_FATFS, UDISK_MOUNTPOINT) == 0)
        KPrintf("Mount FAT on Udisk successful.\n");
    else
        KPrintf("Mount FAT on Udisk failed.\n");
}

void UsbUnmountFileSystem()
{
    UnmountFileSystem(UDISK_MOUNTPOINT);
}
#endif

static uint32 UsbHostOpen(void *dev)
{
    return EOK;
}

static uint32 UsbHostClose(void *dev)
{
    return EOK;
}

static uint32 UsbHostRead(void *dev, struct BusBlockReadParam *read_param)
{
    usb_status_t status;

    status = USB_HostMsdReadApi(&g_MsdCommandInstance, (uint8 *)read_param->buffer, read_param->pos, USB_SINGLE_BLOCK_SIZE, read_param->size);
    if (kStatus_USB_Success == status) {
        return read_param->size;
    }

    return 0;
}

static uint32 UsbHostWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    usb_status_t status;

    status = USB_HostMsdWriteApi(&g_MsdCommandInstance, (const uint8 *)write_param->buffer, write_param->pos, USB_SINGLE_BLOCK_SIZE, write_param->size);
    if (kStatus_USB_Success == status) {
        return write_param->size;
    }

    return 0;
}

/*manage the usb device operations*/
static const struct UsbDevDone dev_done =
{
    .open = UsbHostOpen,
    .close = UsbHostClose,
    .write = UsbHostWrite,
    .read = UsbHostRead,
};

static void UsbHostTask(void* parameter)
{
    while (1) {
        UsbHostTaskFn(g_HostHandle);
        USB_HostMsdTask(&g_MsdCommandInstance);
    }
}

/*Init usb host bus、driver*/
static int BoardUsbBusInit(struct UsbBus *usb_bus, struct UsbDriver *usb_driver)
{
    x_err_t ret = EOK;

    /*Init the usb bus */
    ret = UsbBusInit(usb_bus, USB_BUS_NAME);
    if (EOK != ret) {
        KPrintf("board_usb_init UsbBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the usb driver*/
    ret = UsbDriverInit(usb_driver, USB_DRIVER_NAME);
    if (EOK != ret){
        KPrintf("board_usb_init UsbDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the usb driver to the usb bus*/
    ret = UsbDriverAttachToBus(USB_DRIVER_NAME, USB_BUS_NAME);
    if (EOK != ret) {
        KPrintf("board_usb_init USEDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

/*Attach the usb device to the usb bus*/
static int BoardUsbDevBend(void)
{
    x_err_t ret = EOK;
    static struct UsbHardwareDevice usb_device;
    memset(&usb_device, 0, sizeof(struct UsbHardwareDevice));

    usb_device.dev_done = &dev_done;

    ret = USBDeviceRegister(&usb_device, NONE, USB_DEVICE_NAME);
    if (EOK != ret) {
        KPrintf("USBDeviceRegister device %s error %d\n", USB_DEVICE_NAME, ret);
        return ERROR;
    }

    ret = USBDeviceAttachToBus(USB_DEVICE_NAME, USB_BUS_NAME);
    if (EOK != ret) {
        KPrintf("USBDeviceAttachToBus device %s error %d\n", USB_DEVICE_NAME, ret);
        return ERROR;
    }

    return ret;
}

/*RT1052 BOARD USB INIT*/
int Imxrt1052HwUsbHostInit(void)
{
    x_err_t ret = EOK;
    int32 usb_host_task = 0;

    static struct UsbBus usb_bus;
    memset(&usb_bus, 0, sizeof(struct UsbBus));

    static struct UsbDriver usb_driver;
    memset(&usb_driver, 0, sizeof(struct UsbDriver));

    UsbHostApplicationInit();

    ret = BoardUsbBusInit(&usb_bus, &usb_driver);
    if (EOK != ret) {
        KPrintf("BoardUsbBusInit error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardUsbDevBend();
    if (EOK != ret) {
        KPrintf("BoardUsbDevBend error ret %u\n", ret);
        return ERROR;
    }

    usb_host_task = KTaskCreate("usbh", UsbHostTask, NONE,
                           USB_HOST_STACK_SIZE, 8);
    if(usb_host_task < 0) {		
		KPrintf("usb_host_task create failed ...%s %d.\n", __FUNCTION__,__LINE__);
		return ERROR;
	}

    StartupKTask(usb_host_task);

    return ret;
}

#endif
