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
 * @file plc_socket.h
 * @brief Demo for PLC socket communication function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.03.16
 */

#ifndef __PLC_SOCKET_H_
#define __PLC_SOCKET_H_

#define PLC_BIN_CMD_LEN 512

// for plc socket test bin commands
typedef struct
{
    uint16_t delay_ms;
    uint8_t cmd_len;
    uint8_t cmd[PLC_BIN_CMD_LEN];
}PlcBinCmdType;

enum PlcDeviceType {
    PLC_DEV_TYPE_OML = 0,
    PLC_DEV_TYPE_IPC,
    PLC_DEV_TYPE_BRL,
    PLC_DEV_TYPE_SIEMENS,
    PLC_DEV_TYPE_SIEMENS_1200,
    PLC_DEV_TYPE_JF_IPC,
    PLC_DEV_TYPE_HG,
    /* ...... */
    PLC_DEV_TYPE_END,
};

#define PLC_IP_LEN 16
#define PLC_DEV_NAME_LEN 32
#define PLC_RECV_BUF_LEN CTL_FILE_LEN

typedef struct PlcSocketParamStruct{
    char ip[PLC_IP_LEN];
    uint32_t port;
    uint32_t device_type; //PlcDeviceType
    uint32_t socket_type; //UDP or TCP
    char device[PLC_DEV_NAME_LEN];
    uint32_t cmd_num; // command number
    uint32_t recv_len; // receive length
    uint8_t *recv_buf; // receive buffer
}PlcSocketParamType;

//debug command
#define plc_print //KPrintf
#define plc_error KPrintf
#define plc_notice KPrintf

#endif
