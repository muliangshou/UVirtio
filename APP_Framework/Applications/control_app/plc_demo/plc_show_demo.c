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

#include "transform.h"
#include "list.h"

#include "open62541.h"
#include "ua_api.h"
#include "sys_arch.h"
#include "plc_demo.h"


#define PLC_DEMO_NUM 5

struct PlcDevice plc_demo_array[PLC_DEMO_NUM];

typedef struct PlcShowParam
{
    int id;
    char* vector;
    char* model;
    char* product;
} PlcShowParamType;

PlcShowParamType plc_demo_param[PLC_NAME_SIZE] =
{
    {1, "SIEMENS", "S7-1500", "CPU 1512SP-1PN"},
    {2, "SIEMENS", "S7-1200", "CPU 1215C"},
    {3, "SIEMSNS", "S7-200", "CPU SR60"},
    {4, "B&R", "X20", "X20 CP1586"},
    {5, "B&R", "X20", "X20 CP1381"}
};

static char* const channel_type_str[] =
{
    "PLC_Channel",
    "Unknown"
};

extern DoublelistType plcdev_list;
extern DoublelistType ch_linklist;

/**********************************************************************************************************************/

void PlcShowTitle(const char* item_array[])
{
    int i = 0, max_len = 65;
    KPrintf(" %-15s%-15s%-15s%-15s%-20s\n", item_array[0], item_array[1], item_array[2], item_array[3], item_array[4]);

    while(i < max_len)
    {
        i++;

        if(max_len == i)
        {
            KPrintf("-\n");
        }
        else
        {
            KPrintf("-");
        }
    }
}

static ChDrvType ShowChannelFindDriver(struct Channel* ch)
{
    struct ChDrv* driver = NONE;
    DoublelistType* node = NONE;
    DoublelistType* head = &ch->ch_drvlink;

    for(node = head->node_next; node != head; node = node->node_next)
    {
        driver = DOUBLE_LIST_ENTRY(node, struct ChDrv, driver_link);
        return driver;
    }

    return NONE;
}

static void PlcShowDemoInit(void)
{
    static uint8_t init_flag = 0;
    int i;
    PlcDemoChannelDrvInit();

    for(i = 0; i < PLC_DEMO_NUM; i++)
    {
        // register plc device
        plc_demo_array[i].state = CHDEV_INIT;
        snprintf(plc_demo_array[i].name, PLC_NAME_SIZE, "PLC Demo %d", i);
        plc_demo_array[i].info.vendor = plc_demo_param[i].vector;
        plc_demo_array[i].info.model = plc_demo_param[i].model;
        plc_demo_array[i].info.id = plc_demo_param[i].id;
        plc_demo_array[i].info.product = plc_demo_param[i].product;
        plc_demo_array[i].net = PLC_IND_ENET_OPCUA;
    }

    if(init_flag)
        return;
    init_flag = 1;

    for(i = 0; i < PLC_DEMO_NUM; i++)
    {
        if(PlcDevRegister(&plc_demo_array[i], NULL, plc_demo_array[i].name) == EOK)
        {
            PlcDeviceAttachToChannel(plc_demo_array[i].name, PLC_CH_NAME);
        }
    }
}

void PlcShowChannel(void)
{
    ChannelType ch;
    ChDrvType driver;
    ChDevType device;
    int dev_cnt;
    DoublelistType* ch_node = NONE;
    DoublelistType* ch_head = &ch_linklist;
    const char* item_array[] = {"ch_type", "ch_name", "drv_name", "dev_name", "cnt"};
    PlcShowDemoInit();
    PlcShowTitle(item_array);
    ch_node = ch_head->node_next;

    do
    {
        ch = DOUBLE_LIST_ENTRY(ch_node, struct Channel, ch_link);

        if((ch) && (ch->ch_type == CH_PLC_TYPE))
        {
            KPrintf("%s", " ");
            KPrintf("%-15s%-15s",
                    channel_type_str[ch->ch_type],
                    ch->ch_name);

            driver = ShowChannelFindDriver(ch);

            if(driver)
            {
                KPrintf("%-15s", driver->drv_name);
            }
            else
            {
                KPrintf("%-15s", "nil");
            }

            if(ch->haldev_cnt)
            {
                DoublelistType* dev_node = NONE;
                DoublelistType* dev_head = &ch->ch_devlink;
                dev_node = dev_head->node_next;
                dev_cnt = 1;

                while(dev_node != dev_head)
                {
                    device = DOUBLE_LIST_ENTRY(dev_node, struct ChDev, dev_link);

                    if(1 == dev_cnt)
                    {
                        if(device)
                        {
                            KPrintf("%-16s%-4d\n", device->dev_name, dev_cnt);
                        }
                        else
                        {
                            KPrintf("%-16s%-4d\n", "nil", dev_cnt);
                        }
                    }
                    else
                    {
                        KPrintf("%46s", " ");

                        if(device)
                        {
                            KPrintf("%-16s%-4d\n", device->dev_name, dev_cnt);
                        }
                        else
                        {
                            KPrintf("%-16s%-4d\n", "nil", dev_cnt);
                        }
                    }

                    dev_cnt++;
                    dev_node = dev_node->node_next;
                }
            }
            else
            {
                KPrintf("\n");
            }
        }

        ch_node = ch_node->node_next;
    }
    while(ch_node != ch_head);

    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 ShowChannel, PlcShowChannel, Show PLC information);

void PlcShowDev(void)
{
    PlcDeviceType* plc_dev;
    ChDrvType driver;
    ChDevType device;
    DoublelistType* plc_node = NONE;
    DoublelistType* plc_head = &plcdev_list;
    const char* item_array[] = {"device", "vendor", "model", "product", "id"};
    PlcShowDemoInit();
    PlcShowTitle(item_array);
    plc_node = plc_head->node_next;

    do
    {
        plc_dev = DOUBLE_LIST_ENTRY(plc_node, struct PlcDevice, link);

        if(plc_dev)
        {
            KPrintf("%s", " ");
            KPrintf("%-15s%-15s%-15s%-15s%-20d",
                    plc_dev->name,
                    plc_dev->info.vendor,
                    plc_dev->info.model,
                    plc_dev->info.product,
                    plc_dev->info.id);
            KPrintf("\n");
        }

        plc_node = plc_node->node_next;
    }
    while(plc_node != plc_head);

    return;
}


SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 ShowPlc, PlcShowDev, Show PLC information);

