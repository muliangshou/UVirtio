/*
 * Copyright (c) 2022 AIIT XUOS Lab
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
 * @file plc_show_demo.c
 * @brief Demo for PLC information show
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.02.24
 */

#ifndef __PLC_DEMO_H_
#define __PLC_DEMO_H_

#include "plc_channel.h"
#include "plc_device.h"

#define PLC_CH_NAME "PLC"
#define PLC_DRV_NAME "OPCUA"

#define PLC_BUF_SIZE 128

#define PLC_STACK_SIZE  4096
#define PLC_TASK_PRIO   15

extern struct PlcChannel plc_demo_ch;
extern struct PlcDriver plc_demo_drv;
extern struct PlcDevice plc_demo_dev;

void PlcDemoChannelDrvInit(void);

#endif
