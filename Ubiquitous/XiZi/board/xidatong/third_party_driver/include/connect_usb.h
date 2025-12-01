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
* @file connect_usb.h
* @brief define imxrt1052-board usb function and struct
* @version 2.0
* @author AIIT XUOS Lab
* @date 2022-03-15
*/

#ifndef CONNECT_USB_H
#define CONNECT_USB_H

#include <device.h>
#include <fsl_common.h>
#include <usb_host_config.h>
#include <usb_host.h>
#include <usb_host_msd.h>
#include <host_msd_command.h>
#include <usb_phy.h>

#if defined(FS_VFS)
#include <iot-vfs.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROLLER_ID kUSB_ControllerEhci1
#define USB_HOST_INTERRUPT_PRIORITY (3U)

#define USB_HOST_STACK_SIZE 4096

#define USB_SINGLE_BLOCK_SIZE 512

int Imxrt1052HwUsbHostInit(void);

#ifdef __cplusplus
}
#endif

#endif
