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
 * @file plc_control_demo.c
 * @brief Demo for PLC control
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.2.22
 */

#include "transform.h"
#include "open62541.h"
#include "ua_api.h"
#include "sys_arch.h"
#include "plc_demo.h"

#define PLC_NS_FORMAT "n%d,%s"

struct PlcChannel plc_demo_ch;
struct PlcDriver plc_demo_drv;
struct PlcDevice plc_demo_dev;

int plc_test_flag = 0;

PlcCtrlParamType plc_ctrl_param;

UA_NodeId test_nodeid = {4, UA_NODEIDTYPE_NUMERIC, 5};

/******************************************************************************/

void PlcDelay(int sec)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 100000000 * sec; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

// get NodeId from str
void PlcGetTestNodeId(char *str, UA_NodeId *id)
{
    static char node_str[UA_NODE_LEN];
    memset(node_str, 0, sizeof(node_str));

    plc_print("plc: arg %s\n", str);

    if(sscanf(str, PLC_NS_FORMAT, &id->namespaceIndex, node_str) != EOF)
    {
        if(isdigit(node_str[0]))
        {
            id->identifierType = UA_NODEIDTYPE_NUMERIC;
            id->identifier.numeric = atoi(node_str);
            plc_print("ns %d num %d\n", id->namespaceIndex, id->identifier.numeric);
        }
        else
        {
            id->identifierType = UA_NODEIDTYPE_STRING;
            id->identifier.string.length = strlen(node_str);
            id->identifier.string.data = node_str;
            plc_print("ns %d str %s\n", id->namespaceIndex, id->identifier.string.data);
        }
    }
}

void PlcDemoChannelDrvInit(void)
{
    static uint8_t init_flag = 0;
    if(init_flag)
        return;
    init_flag = 1;

    lwip_config_tcp(lwip_ipaddr, lwip_netmask, test_ua_ip);
    PlcChannelInit(&plc_demo_ch, PLC_CH_NAME);
    if(PlcDriverInit(&plc_demo_drv, PLC_DRV_NAME) == EOK)
    {
        PlcDriverAttachToChannel(PLC_DRV_NAME, PLC_CH_NAME);
    }
    memset(&plc_demo_dev, 0, sizeof(plc_demo_dev));
}

static void PlcGetDemoDev(PlcDeviceType *dev, UA_NodeId *id)
{
    // register plc device
    dev->state = CHDEV_INIT;
    strcpy(dev->name, "UA Demo");
    dev->info.product = "CPU 1215C";
    dev->info.vendor = "SIEMENS";
    dev->info.model = "S7-1200";
    dev->info.id = 123;
    dev->net = PLC_IND_ENET_OPCUA;

    // register UA parameter
    if(!dev->priv_data)
    {
        dev->priv_data = (UaParamType*)malloc(sizeof(UaParamType));
    }
    UaParamType* ua_ptr = dev->priv_data;
    memset(ua_ptr, 0, sizeof(UaParamType));
    strcpy(ua_ptr->ua_remote_ip, opc_server_url);
    ua_ptr->act = UA_ACT_ATTR;
    memcpy(&ua_ptr->ua_id, id, sizeof(*id));
}

static void PlcCtrlDemoInit(void)
{
    static uint8_t init_flag = 0;

    PlcDemoChannelDrvInit();

    // register plc device
    PlcGetDemoDev(&plc_demo_dev, &test_nodeid);

    if(init_flag)
    {
        return;
    }
    init_flag = 1;

    if(PlcDevRegister(&plc_demo_dev, NULL, plc_demo_dev.name) != EOK)
    {
        return;
    }
    PlcDeviceAttachToChannel(plc_demo_dev.name, PLC_CH_NAME);
}

void PlcReadUATask(void* arg)
{
    int ret = 0;
    struct PlcOps* ops = NULL;
    char buf[PLC_BUF_SIZE];
    memset(buf, 0, sizeof(buf));
    PlcCtrlDemoInit();
    ops = plc_demo_dev.ops;
    ret = ops->open(&plc_demo_dev);

    if(EOK != ret)
    {
        plc_print("plc: [%s] open failed %#x\n", __func__, ret);
        return;
    }

    ret = ops->read(&plc_demo_dev, buf, PLC_BUF_SIZE);

    if(EOK != ret)
    {
        plc_print("plc: [%s] read failed %x\n", __func__, ret);
    }

    ops->close(&plc_demo_dev);
}

void PlcReadTestShell(int argc, char* argv[])
{
    static char node_str[UA_NODE_LEN];
    memset(node_str, 0, sizeof(node_str));

    if(argc > 1)
    {
        PlcGetTestNodeId(argv[1], &test_nodeid);
    }

    sys_thread_new("plc read", PlcReadUATask, NULL, PLC_STACK_SIZE, PLC_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 PlcRead, PlcReadTestShell, Read PLC);

void PlcWriteUATask(void* arg)
{
    int ret = 0;
    struct PlcOps* ops = NULL;
    char buf[PLC_BUF_SIZE];
    memset(buf, 0, sizeof(buf));

    PlcCtrlDemoInit();
    ops = plc_demo_dev.ops;
    ret = ops->open(&plc_demo_dev);

    if(EOK != ret)
    {
        plc_print("plc: [%s] open failed %#x\n", __func__, ret);
        return;
    }

    ret = ops->write(&plc_demo_dev, arg, PLC_BUF_SIZE);

    if(EOK != ret)
    {
        plc_print("plc: [%s] write failed\n", __func__);
    }

    ops->close(&plc_demo_dev);
}

void PlcWriteTestShell(int argc, char* argv[])
{
    static char node_str[UA_NODE_LEN];
    static char val_param[UA_NODE_LEN];
    memset(node_str, 0, sizeof(node_str));
    memset(val_param, 0, sizeof(val_param));

    if(argc > 1)
    {
        PlcGetTestNodeId(argv[1], &test_nodeid);
    }

    if(argc > 2)
    {
        strcpy(val_param, argv[2]);
        plc_print("write value %s\n", val_param);
    }

    sys_thread_new("plc write", PlcWriteUATask, val_param, PLC_STACK_SIZE, PLC_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 PlcWrite, PlcWriteTestShell, Read PLC);

// test motor
// clear parameter
// PlcWrite n4,2 0b
// PlcWrite n4,3 0b
// PlcWrite n4,4 0b
// PlcWrite n4,5 0b
//
// enable
// PlcWrite n4,2 1b
//
// set rotate speed
// PlcWrite n4,3 50
//
// positive turn
// PlcWrite n4,4 1b
//
// reversal turn
// PlcWrite n4,5 1b

static int plc_test_speed = 50;
static int plc_test_dir = 1; // direction positive: 1 reversal: 0

void PlcMotorTestTask(void* arg)
{
    //support node id
    char *test_nodeid[] = {"n4,2", "n4,3", "n4,4", "n4,5", "n4,7"};
    // enable -> speed -> positive dir or inversal dir -> stop -> enable
    char test_sort[] = {0, 4, 2, 1, 0};
    char test_cmd[][4] = {"1b", "50", "1b", "1b", "0b"};
    char *test_notice[] = {"Enable Motor", "Set Speed", "Set Forward", "Set Reverse", "Stop Motor"};

    int ret = 0;
    struct PlcOps* ops = NULL;
    char buf[PLC_BUF_SIZE];
    memset(buf, 0, sizeof(buf));

    PlcCtrlDemoInit();
    ops = plc_demo_dev.ops;
    ret = ops->open(&plc_demo_dev);

    if(EOK != ret)
    {
        plc_print("plc: [%s] open failed %#x\n", __func__, ret);
        return;
    }

    UaParamType* ua_ptr = plc_demo_dev.priv_data;

    // initialize step
    for(int i = 0; i < 5; i++)
    {
        plc_print("###\n### Clear %s\n###\n", test_notice[i]);
        PlcGetTestNodeId(test_nodeid[i], &ua_ptr->ua_id);
        ret = ops->write(&plc_demo_dev, "0b", PLC_BUF_SIZE);
        if(EOK != ret)
        {
            plc_print("plc: [%s] %d write failed\n", __func__, __LINE__);
        }
        PlcDelay(1);
    }

    if(plc_test_speed != 50)
    {
        snprintf(test_cmd[1], 4, "%d", plc_test_speed);
    }

    if(plc_test_dir == 0) // if not postive, next running
        test_sort[2] = 3;

    for(int i = 0; i < sizeof(test_sort)/sizeof(test_sort[0]); i++)
    {
        PlcGetTestNodeId(test_nodeid[test_sort[i]], &ua_ptr->ua_id);
        plc_print("###\n### %s\n###\n", test_notice[i]);
        ret = ops->write(&plc_demo_dev, test_cmd[i], PLC_BUF_SIZE);
        if(EOK != ret)
        {
            plc_print("plc: [%s] %d write failed\n", __func__, __LINE__);
        }
        PlcDelay(1);
        if(i == 2) // postive
        {
            PlcDelay(10);
        }
    }
    ops->close(&plc_demo_dev);
    plc_test_flag = 0;
}

// get parameter from
void PlcGetMotorParam(char *str)
{
    static char node_str[UA_NODE_LEN];
    memset(node_str, 0, sizeof(node_str));

    plc_print("plc: arg %s\n", str);

    sscanf(str, "speed=%d", &plc_test_speed);
    sscanf(str, "dir=%d", &plc_test_dir);
    plc_print("speed is %d\n", plc_test_speed);
    plc_print("dir is %d\n", plc_test_dir);
}

void PlcMotorTestShell(int argc, char* argv[])
{
    if(plc_test_flag)
    {
        plc_print("PLC Motor testing!\n");
        return;
    }
    plc_test_flag = 1;

    if(argc > 1)
    {
        for(int i = 0; i < argc; i++)
        {
            PlcGetMotorParam(argv[i]);
        }
    }

    sys_thread_new("plc motor", PlcMotorTestTask, NULL, PLC_STACK_SIZE, PLC_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
                 PlcMotorTest, PlcMotorTestShell, Run motor);

