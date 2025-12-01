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
* @file:    test_adc.c
* @brief:   a application of adc function
* @version: 1.1
* @author:  AIIT XUOS Lab
* @date:    2022/1/7
*/

#include <stdio.h>
#include <string.h>
#include <transform.h>

void test_adc()
{
    int adc_fd;
    uint8 adc_channel = 0x0;
    uint16 adc_sample, adc_value_decimal = 0;
    float adc_value;

    adc_fd = PrivOpen(ADC_DEV_DRIVER, O_RDWR);
    if (adc_fd < 0) {
        printf("open adc fd error %d\n", adc_fd);
        return;
    }

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = ADC_TYPE;
    ioctl_cfg.args = &adc_channel;
    if (0 != PrivIoctl(adc_fd, OPE_CFG, &ioctl_cfg)) {
        printf("ioctl adc fd error %d\n", adc_fd);
        PrivClose(adc_fd);
        return;
    }

    PrivRead(adc_fd, &adc_sample, 2);

    adc_value = (float)adc_sample * (3.3 / 4096);

    adc_value_decimal = (adc_value - (uint16)adc_value) * 1000;

    printf("adc sample %u value integer %u decimal %u\n", adc_sample, (uint16)adc_value, adc_value_decimal);

    PrivClose(adc_fd);

    return;
}

 SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                                                 adc, test_adc, read 3.3 voltage data from adc);
