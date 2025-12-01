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
 * @file plc_socket.c
 * @brief Demo for PLC socket communication function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.03.16
 */

#include "transform.h"
#include "plc_socket.h"
#include "sys_arch.h"
#include "lwip/sockets.h"
#include "control_file.h"

// max support plc socket test commands number
#define PLC_SOCK_CMD_NUM CTL_CMD_NUM
#define PLC_SOCK_TIMEOUT 50000

// for saving PLC command index
int plc_cmd_index = 0;

// only for test
#define SUPPORT_PLC_SIEMENS

//siemens test
PlcBinCmdType TestPlcCmd[PLC_SOCK_CMD_NUM] = {0};

//Test information
//SIEMENS ip: 192.168.250.9 port: 102
//S7-200  ip: 192.168.250.8 port: 102
//S7-1200 ip: 192.168.250.6 port: 102
//OML     ip: 192.168.250.3 port: 9600

PlcSocketParamType plc_socket_demo_data = {
#ifdef SUPPORT_PLC_SIEMENS
    .ip = {192, 168, 250, 6},
    .port = 102,
    .device_type = PLC_DEV_TYPE_SIEMENS,
    .socket_type = SOCK_STREAM,
    .cmd_num = 3,
#else
    .ip = {192, 168, 250, 3},
    .port = 9600,
    .device_type = PLC_DEV_TYPE_OML,
    .socket_type = SOCK_DGRAM,
    .cmd_num = 1,
#endif
    .recv_len = PLC_RECV_BUF_LEN,
    .recv_buf = NULL,
};

#define OML_HEADER_LEN 78
#define CHECK_OML_HEADER(_s) ((0xC0 == *(_s)) && (0x00 == *(_s + 1)) && (0x02 == *(_s + 2)) && (0x00 == *(_s + 3)))

/******************************************************************************/

static void plc_print_array(char *title, int size, uint8_t *cmd)
{
    lw_notice("%s : %d - ", title, size);
    for(int i = 0; i < size; i++)
    {
        lw_notice(" %#x",  cmd[i]);
    }
    lw_notice("\n");
}

static void *PlcSocketStart(void *arg)
{
    int fd = -1;
    int timeout, recv_len;
    struct sockaddr_in sock_addr;
    socklen_t addr_len = sizeof(struct sockaddr_in);
    PlcSocketParamType *param = (PlcSocketParamType *)&plc_socket_demo_data;

    plc_print("start %d.%d.%d.%d:%d dev %d sock %d\n",
        param->ip[0],
        param->ip[1],
        param->ip[2],
        param->ip[3],
        param->port,
        param->device_type,
        param->socket_type);

    param->recv_len = PLC_RECV_BUF_LEN;

    //malloc memory
    param->recv_buf = (char *)malloc(param->recv_len);
    if (param->recv_buf == NULL)
    {
        plc_error("No memory\n");
        return NULL;
    }

    fd = socket(AF_INET, param->socket_type, 0);
    if (fd < 0)
    {
        plc_error("Socket error %d\n", param->socket_type);
        free(param->recv_buf);
        return NULL;
    }

    plc_print("start %d.%d.%d.%d:%d\n", param->ip[0], param->ip[1], param->ip[2], param->ip[3], param->port);

    sock_addr.sin_family = AF_INET;
    sock_addr.sin_port = htons(param->port);
    sock_addr.sin_addr.s_addr = PP_HTONL(LWIP_MAKEU32(param->ip[0], param->ip[1], param->ip[2], param->ip[3]));
    memset(&(sock_addr.sin_zero), 0, sizeof(sock_addr.sin_zero));

    if (connect(fd, (struct sockaddr *)&sock_addr, sizeof(struct sockaddr)) < 0)
    {
        plc_error("Unable to connect\n");
        closesocket(fd);
        free(param->recv_buf);
        return NULL;
    }

    lw_notice("client %s connected\n", inet_ntoa(sock_addr.sin_addr));

    for(int i = 0; i < param->cmd_num; i ++)
    {
        PlcBinCmdType *cmd = &TestPlcCmd[i];
        sendto(fd, cmd->cmd, cmd->cmd_len, 0, (struct sockaddr*)&sock_addr, addr_len);
        plc_print_array("Send cmd", cmd->cmd_len, cmd->cmd);

        MdelayKTask(cmd->delay_ms);
        timeout = PLC_SOCK_TIMEOUT;
        memset(param->recv_buf, 0, param->recv_len);
        while(timeout --)
        {
            recv_len = recvfrom(fd, param->recv_buf, param->recv_len, 0, (struct sockaddr *)&sock_addr, &addr_len);
            if(recv_len > 0)
            {
                if(param->device_type == PLC_DEV_TYPE_OML)
                {
                    if((recv_len == OML_HEADER_LEN) && (CHECK_OML_HEADER(param->recv_buf)))
                    {
                        lw_notice("This is Oml package!!!\n");
                    }
                }
                lw_notice("Receive from : %s\n", inet_ntoa(sock_addr.sin_addr));
                plc_print_array("Receive data", recv_len, param->recv_buf);
                break;
            }
        }
    }

    closesocket(fd);
    free(param->recv_buf);
    return NULL;
}

void PlcGetParamCmd(char *cmd)
{
    const char s[2] = ",";
    char *token;
    uint16_t cmd_index = 0;
    char bin_cmd[PLC_BIN_CMD_LEN] = {0};
    token = strtok(cmd, s);
    while(token != NULL)
    {
        sscanf(token, "%x", &bin_cmd[cmd_index]);
        plc_print("%d - %s %d\n", cmd_index, token, bin_cmd[cmd_index]);
        token = strtok(NULL, s);
        cmd_index ++;
    }
    TestPlcCmd[plc_cmd_index].cmd_len = cmd_index;
    memcpy(TestPlcCmd[plc_cmd_index].cmd, bin_cmd, cmd_index);
    plc_print("get %d cmd len %d\n", plc_cmd_index, TestPlcCmd[plc_cmd_index].cmd_len);
    plc_cmd_index ++;
    plc_socket_demo_data.cmd_num = plc_cmd_index;
}

void PlcShowUsage(void)
{
    plc_notice("------------------------------------\n");
    plc_notice("PlcSocket [ip].[ip].[ip].[ip]:[port]\n");
    plc_notice("PlcSocket support other param:\n");
    plc_notice("plc=[] 0: OML 1:SIEMENS\n");
    plc_notice("tcp=[] 0: udp 1:tcp\n");
    plc_notice("ip=[ip.ip.ip.ip]\n");
    plc_notice("port=port\n");
    plc_notice("file: use %s\n", PLC_SOCK_FILE_NAME);
    plc_notice("------------------------------------\n");
}

#if defined(MOUNT_SDCARD) && defined(LIB_USING_CJSON)
void PlcGetParamFromFile(char *file_name)
{
    PlcSocketParamType *param = &plc_socket_demo_data;

    char *file_buf = malloc(CTL_FILE_LEN);
    if(file_buf == NULL)
    {
        plc_error("No enough buffer %d\n", CTL_FILE_LEN);
        return;
    }
    memset(file_buf, 0, CTL_FILE_LEN);

    if(CtlFileReadWithFilename(file_name, CTL_FILE_LEN, file_buf) != EOK)
    {
        plc_error("Can't open file %s\n", file_name);
        //try again default file
        if(strcmp(file_name, PLC_SOCK_FILE_NAME) != 0)
        {
            if(CtlFileReadWithFilename(PLC_SOCK_FILE_NAME, CTL_FILE_LEN, file_buf) != EOK)
            {
                plc_error("Can't open file %s\n", file_name);
                return;
            }
        }
        else
        {
            return;
        }
    }
    CtlParseJsonData(file_buf);

    memcpy(param->ip, ctl_file_param.ip, 4);
    param->port = ctl_file_param.port;
    param->cmd_num = ctl_file_param.cmd_num;
    param->socket_type = ctl_file_param.tcp ? SOCK_STREAM : SOCK_DGRAM;

    for(int i = 0; i < param->cmd_num; i++)
    {
        TestPlcCmd[i].cmd_len = ctl_file_param.cmd_len[i];
        memcpy(TestPlcCmd[i].cmd, ctl_file_param.cmd[i], TestPlcCmd[i].cmd_len);
    }

    plc_print("ip: %d.%d.%d.%d\n", param->ip[0], param->ip[1], param->ip[2], param->ip[3]);
    plc_print("port: %d", param->port);
    plc_print("tcp: %d", param->socket_type);
    plc_print("cmd number: %d\n", param->cmd_num);

    for(int i = 0; i < param->cmd_num; i++)
    {
        plc_print_array("cmd", TestPlcCmd[i].cmd_len, TestPlcCmd[i].cmd);
    }
    free(file_buf);
}

#endif

void PlcCheckParam(int argc, char *argv[])
{
    int i;
    PlcSocketParamType *param = &plc_socket_demo_data;
    plc_cmd_index = 0;

    for(i = 0; i < argc; i++)
    {
        char *str = argv[i];
        int is_tcp = 0;
        char cmd_str[PLC_BIN_CMD_LEN] = {0};

        plc_print("check %d %s\n", i, str);

#if defined(MOUNT_SDCARD) && defined(LIB_USING_CJSON)
        if(strncmp(str, "file", 4) == 0)
        {
            char file_name[CTL_FILE_NAME_LEN] = {0};
            if(sscanf(str, "file=%s", file_name) == EOF)
            {
                strcpy(file_name, PLC_SOCK_FILE_NAME);
            }
            plc_notice("get %s parameter file %s\n", str, file_name);
            PlcGetParamFromFile(file_name);
            return;
        }
#endif
        if(sscanf(str, "ip=%d.%d.%d.%d",
            &param->ip[0],
            &param->ip[1],
            &param->ip[2],
            &param->ip[3]) == 4)
        {
            plc_print("find ip %d %d %d %d\n", param->ip[0], param->ip[1], param->ip[2], param->ip[3]);
            continue;
        }

        if(sscanf(str, "port=%d", &param->port) == 1)
        {
            plc_print("find port %d\n", param->port);
            continue;
        }

        if(sscanf(str, "tcp=%d", &is_tcp) == 1)
        {
            plc_print("find tcp %d\n", is_tcp);
            param->socket_type = is_tcp ? SOCK_STREAM:SOCK_DGRAM;
            continue;
        }

        if(sscanf(str, "plc=%d", &param->device_type) == 1)
        {
            plc_print("find device %d\n", param->device_type);
            continue;
        }

        if(sscanf(str, "cmd=%s", cmd_str) == 1)
        {
            plc_print("find cmd %s\n", cmd_str);
            PlcGetParamCmd(cmd_str);
            continue;
        }
    }

    if(argc >= 2)
    {
        if(sscanf(argv[1], "%d.%d.%d.%d:%d",
            &param->ip[0],
            &param->ip[1],
            &param->ip[2],
            &param->ip[3],
            &param->port) != EOF)
        {
            return;
        }

        if(sscanf(argv[1], "%d.%d.%d.%d",
            &param->ip[0],
            &param->ip[1],
            &param->ip[2],
            &param->ip[3]) != EOF)
        {
            return;
        }
    }
    else
    {
        PlcShowUsage();
    }
}

void PlcSocketTask(int argc, char *argv[])
{
    int result = 0;
    pthread_t th_id;

    pthread_attr_t attr;
    attr.schedparam.sched_priority = LWIP_DEMO_TASK_PRIO;
    attr.stacksize = LWIP_TASK_STACK_SIZE;
    PlcSocketParamType *param = &plc_socket_demo_data;

    PlcCheckParam(argc, argv);

    lwip_config_net(lwip_ipaddr, lwip_netmask, param->ip);
    PrivTaskCreate(&th_id, &attr, PlcSocketStart, param);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     PlcSocket, PlcSocketTask, Test PLC Socket);

