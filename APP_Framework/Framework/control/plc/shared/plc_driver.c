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
* @file drv_plc.c
* @brief register plc drv function using bus driver framework
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-01-24
*/

#include "transform.h"
#include "plc_channel.h"
#include "plc_device.h"

static DoublelistType plcdrv_linklist;

/******************************************************************************/

/*Create the driver linklist*/
static void PlcDrvLinkInit()
{
    AppInitDoubleList(&plcdrv_linklist);
}

ChDrvType PlcDriverFind(const char *drv_name, enum ChDrvType_e drv_type)
{
    CHECK_CH_PARAM(drv_name);

    struct ChDrv *driver = NONE;

    DoublelistType *node = NONE;
    DoublelistType *head = &plcdrv_linklist;

    for (node = head->node_next; node != head; node = node->node_next) {
        driver = DOUBLE_LIST_ENTRY(node, struct ChDrv, driver_link);
        if ((!strcmp(driver->drv_name, drv_name)) && (drv_type == driver->driver_type)) {
            return driver;
        }
    }

    KPrintf("PlcDriverFind cannot find the %s driver.return NULL\n", drv_name);
    return NONE;
}

int PlcDriverRegister(struct ChDrv *driver)
{
    CHECK_CH_PARAM(driver);

    int ret = EOK;
    static uint8_t driver_link_flag = 0;

    if (!driver_link_flag) {
        PlcDrvLinkInit();
        driver_link_flag = 1;
    }

    AppDoubleListInsertNodeAfter(&plcdrv_linklist, &(driver->driver_link));

    return ret;
}

