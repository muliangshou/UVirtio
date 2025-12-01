/*
* Copyright (c) 2021 AIIT XUOS Lab
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
* @file connect_adc.h
* @brief define ok1052-c board adc function and struct
* @version 1.0
* @author AIIT XUOS Lab
* @date 2021-12-28
*/

#ifndef CONNECT_ADC_H
#define CONNECT_ADC_H

#include <device.h>

struct Imxrt1052HwAdc
{
    void *ADCx;
    uint8 adc_channel;
};

int Imxrt1052HwAdcInit(void);

#endif
