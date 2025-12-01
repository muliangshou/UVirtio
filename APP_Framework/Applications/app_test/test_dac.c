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
* @file:    test_dac.c
* @brief:   a application of dac function
* @version: 2.0
* @author:  AIIT XUOS Lab
* @date:    2022/1/11
*/

#include <stdio.h>
#include <string.h>
#include <transform.h>

void test_dac()
{
    int dac_fd;
    uint16 dac_set_value = 800;
    uint16 dac_sample, dac_value_decimal = 0;
    float dac_value;

    dac_fd = PrivOpen(DAC_DEV_DRIVER, O_RDWR);
    if (dac_fd < 0) {
        KPrintf("open dac fd error %d\n", dac_fd);
        return;
    }

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = DAC_TYPE;
    ioctl_cfg.args = &dac_set_value;
    if (0 != PrivIoctl(dac_fd, OPE_CFG, &ioctl_cfg)) {
        KPrintf("ioctl dac fd error %d\n", dac_fd);
        PrivClose(dac_fd);
        return;
    }

    PrivRead(dac_fd, &dac_sample, 2);

    dac_value = (float)dac_sample * (3.3 / 4096);//Vref+ need to be 3.3V

    dac_value_decimal = (dac_value - (uint16)dac_value) * 1000;

    printf("dac sample %u value integer %u decimal %u\n", dac_sample, (uint16)dac_value, dac_value_decimal);

    PrivClose(dac_fd);

    return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                                                test_dac, test_dac, set digital data to dac);
