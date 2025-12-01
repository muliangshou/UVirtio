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
 * @file control_file.c
 * @brief control relative file operation
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022-03-17
 */

#include <stdio.h>
#include "cJSON.h"
#include "transform.h"
#include "control_file.h"

//json file parameter for PLC socket communication as below:
//{
//   "ip": "192.168.250.6",
//   "port": 102,
//   "cmd": [3, 0, 0, 22, 17, 224, 0, 0, 2, 200, 0, 193, 2, 2, 1, 194, 2, 2, 1, 192, 1, 10],
//   "cmd1": [3, 0, 0, 25, 2, 240, 128, 50, 1, 0, 0, 0, 13, 0, 8, 0, 0, 240, 0, 0, 1, 0, 1, 0, 240], \r\n" \
//   "cmd2": [3, 0, 0, 31, 2, 240, 128, 50, 1, 0, 0, 51, 1, 0, 14, 0, 0, 4, 1, 18, 10, 16,  2, 0, 210, 0, 52, 132, 0, 0, 0]\r\n" \
//}"

#define TEST_PLC_JSON_TXT \
"{ \r\n"\
"   \"ip\": \"192.168.250.6\", \r\n"\
"   \"port\": 102, \r\n"\
"   \"tcp\": 1, \r\n"\
"   \"cmd\": [3, 0, 0, 22, 17, 224, 0, 0, 2, 200, 0, 193, 2, 2, 1, 194, 2, 2, 1, 192, 1, 10], \r\n"\
"   \"cmd1\": [3, 0, 0, 25, 2, 240, 128, 50, 1, 0, 0, 0, 13, 0, 8, 0, 0, 240, 0, 0, 1, 0, 1, 0, 240], \r\n" \
"   \"cmd2\": [3, 0, 0, 31, 2, 240, 128, 50, 1, 0, 0, 51, 1, 0, 14, 0, 0, 4, 1, 18, 10, 16,  2, 0, 210, 0, 52, 132, 0, 0, 0]\r\n" \
"}"


CtlPlcSockParamType ctl_file_param;

FILE *CtlFileInit(char *file)
{
    FILE *fd = NULL;

#ifdef MOUNT_SDCARD
    // SD card mount flag 1: OK
    if(sd_mount_flag == 0)
    {
        ctl_error("SD card mount failed\n");
        return NULL;
    }

    fd = fopen(file, "a+");
    if(fd == NULL)
    {
        ctl_error("open file %s failed\n", file);
    }

#endif
    return fd;
}

void CtlFileClose(FILE *fd)
{
    fclose(fd);
}

void CtlFileRead(FILE *fd, int size, char *buf)
{
    fseek(fd, 0, SEEK_SET);
    fread(buf, size, 1, fd);
    ctl_print("read file %d: %.100s\n", size, buf);
}

void CtlFileWrite(FILE *fd, int size, char *buf)
{
    size_t write_size = 0;
    write_size = fwrite(buf, strlen(buf) + 1, 1, fd);
    ctl_print("write size %d: %s\n", size, buf);
}

int CtlFileReadWithFilename(char *file, int size, char *buf)
{
    FILE *fd;
    fd = fopen(file, "r");
    if(fd == NULL)
    {
        ctl_error("open file %s failed\n", file);
        return EEMPTY;
    }

    fseek(fd, 0, SEEK_SET);
    fread(buf, size, 1, fd);
    ctl_print("read file %d: %.100s\n", size, buf);
    return EOK;
}

void CtlCreateFileTest(void)
{
    FILE *fd = CtlFileInit(PLC_SOCK_FILE_NAME);
    if(fd == NULL)
        return;
    char *file_buf = TEST_PLC_JSON_TXT;
    CtlFileWrite(fd, strlen(file_buf), file_buf);
    CtlFileClose(fd);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
    CtlCreateFile, CtlCreateFileTest, Test control file);

#ifdef LIB_USING_CJSON

void CtlParseJsonArray(cJSON *dat, int *cmd_len, char *cmd)
{
    int len, i;
    if(cJSON_IsArray(dat))
    {
        len = cJSON_GetArraySize(dat);
        ctl_print("json cmd %d\n", len);

        for(i = 0; i < len; i++)
        {
            cJSON *cmd_val = cJSON_GetArrayItem(dat, i);
            if(NULL == cmd_val)
                continue;
            ctl_print("0x%x ", cmd_val->valueint);
            cmd[i] = cmd_val->valueint;
        }
        *cmd_len = len;
        ctl_print("\n");
    }
}

void CtlParseJsonData(char *buf)
{
    cJSON *file_dat = NULL;
    cJSON *ip_dat = NULL;
    cJSON *port_dat = NULL;
    cJSON *tcp_dat = NULL;
    cJSON *cmd_dat = NULL;
    char cmd_title[10] = {"cmd"};
    CtlPlcSockParamType *file_param = &ctl_file_param;

    file_dat = cJSON_Parse(buf);
    if(file_dat == NULL)
    {
        ctl_error("ctrl parse failed\n");
        return;
    }

    ip_dat = cJSON_GetObjectItem(file_dat, "ip");
    port_dat = cJSON_GetObjectItem(file_dat, "port");
    tcp_dat = cJSON_GetObjectItem(file_dat, "tcp");

    ctl_print(" ip  : %s\n", ip_dat->valuestring);
    sscanf(ip_dat->valuestring, "%d.%d.%d.%d", &file_param->ip[0],
        &file_param->ip[1],
        &file_param->ip[2],
        &file_param->ip[3]);

    ctl_print(" port: %s %d\n", ip_dat->string, port_dat->valueint);
    file_param->port = port_dat->valueint;
    file_param->tcp = tcp_dat->valueint;
    file_param->cmd_num = 0;

    for(int i = 0; i < CTL_CMD_NUM; i++)
    {
        cmd_dat = cJSON_GetObjectItem(file_dat, cmd_title);
        if(!cmd_dat)
            break;
        CtlParseJsonArray(cmd_dat, &file_param->cmd_len[i], file_param->cmd[i]);
        snprintf(cmd_title, sizeof(cmd_title), "cmd%d", ++file_param->cmd_num);
    }

    cJSON_Delete(file_dat);
}

void CtlParseFileTest(void)
{
    //for PLC socket parameter file
    FILE *fd = CtlFileInit(PLC_SOCK_FILE_NAME);
    if(fd == NULL)
    {
        ctl_error("ctl get file %s failed\n", PLC_SOCK_FILE_NAME);
        return;
    }

    char *file_buf = malloc(CTL_FILE_LEN);

    if(file_buf == NULL)
    {
        ctl_error("ctl malloc failed\n");
        return;
    }
    memset(file_buf, 0, CTL_FILE_LEN);
    CtlFileRead(fd, CTL_FILE_LEN, file_buf);
    CtlFileClose(fd);
    CtlParseJsonData(file_buf);
    free(file_buf);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
    CtlParseFile, CtlParseFileTest, Parse control file);

#endif

