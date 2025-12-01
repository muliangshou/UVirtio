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
 * @file control_file.h
 * @brief control relative API
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022-03-17
 */

#ifndef __CONTROL_FILE_H_
#define __CONTROL_FILE_H_

#define CTL_FILE_LEN 1000 // control file size
#define CTL_CMD_NUM 10 // support command number
#define CTL_CMD_LEN 100 // control command length
#define CTL_IP_LEN 32 // IP address length
#define CTL_FILE_NAME_LEN 100 // file name length

#define PLC_SOCK_FILE_NAME "/plc/socket_param.json"

#define ctl_print //KPrintf
#define ctl_error KPrintf

// for running plc socket
typedef struct CtlPlcSockParamStruct
{
    char ip[CTL_IP_LEN];
    int port;
    int tcp; // 1: TCP 0: UDP
    int cmd_num; //command number
    int cmd_len[CTL_CMD_NUM]; // command length
    char cmd[CTL_CMD_NUM][CTL_CMD_LEN];
}CtlPlcSockParamType;

extern CtlPlcSockParamType ctl_file_param;
extern int sd_mount_flag;

FILE *CtlFileInit(char *file);
void CtlFileClose(FILE *fd);
void CtlFileRead(FILE *fd, int size, char *buf);
void CtlFileWrite(FILE *fd, int size, char *buf);
int CtlFileReadWithFilename(char *file_name, int size, char *buf);

#ifdef LIB_USING_CJSON
void CtlParseJsonData(char *buf);
#endif
#endif

