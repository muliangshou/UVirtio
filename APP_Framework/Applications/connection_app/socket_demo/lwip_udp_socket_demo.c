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
* @file lwip_udp_socket_demo.c
* @brief UDP demo based on LwIP
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-21
*/
#include <transform.h>
#include "sys_arch.h"
#include "lwip/sockets.h"

#define UDP_BUF_SIZE 65536

char udp_socket_ip[] = {192, 168, 250, 252};
u16_t udp_socket_port = LWIP_LOCAL_PORT;

/*****************************************************************************/

static void UdpSocketRecvTask(void *arg)
{
    int fd = -1;
    char *recv_buf;
    struct sockaddr_in udp_addr, server_addr;
    int recv_len;
    socklen_t addr_len;

    while(1)
    {
        recv_buf = (char *)malloc(UDP_BUF_SIZE);
        if(recv_buf == NULL)
        {
            lw_error("No memory\n");
            continue;
        }

        fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(fd < 0)
        {
            lw_error("Socket error\n");
            free(recv_buf);
            continue;
        }

        udp_addr.sin_family = AF_INET;
        udp_addr.sin_addr.s_addr = INADDR_ANY;
        udp_addr.sin_port = htons(udp_socket_port);
        memset(&(udp_addr.sin_zero), 0, sizeof(udp_addr.sin_zero));

        if(bind(fd, (struct sockaddr *)&udp_addr, sizeof(struct sockaddr)) == -1)
        {
            lw_error("Unable to bind\n");
            closesocket(fd);
            free(recv_buf);
            continue;
        }

        lw_notice("UDP bind sucess, start to receive.\n");
        lw_notice("\n\nLocal Port:%d\n\n", udp_socket_port);

        while(1)
        {
            memset(recv_buf, 0, UDP_BUF_SIZE);
            recv_len = recvfrom(fd, recv_buf, UDP_BUF_SIZE, 0, (struct sockaddr *)&server_addr, &addr_len);
            if(recv_len > 0)
            {
                lw_notice("Receive from : %s\n", inet_ntoa(server_addr.sin_addr));
                lw_notice("Receive data : %s\n\n", recv_buf);
            }
            sendto(fd, recv_buf, recv_len, 0, (struct sockaddr*)&server_addr, addr_len);
        }

        closesocket(fd);
        free(recv_buf);
    }
}

void UdpSocketRecvTest(int argc, char *argv[])
{
    if(argc >= 2)
    {
        lw_notice("lw: [%s] target ip %s\n", __func__, argv[1]);
        if(sscanf(argv[1], "%d.%d.%d.%d:%d", &udp_socket_ip[0], &udp_socket_ip[1], &udp_socket_ip[2], &udp_socket_ip[3], &udp_socket_port) == EOK)
        {
            sscanf(argv[1], "%d.%d.%d.%d", &udp_socket_ip[0], &udp_socket_ip[1], &udp_socket_ip[2], &udp_socket_ip[3]);
        }
    }

    lwip_config_tcp(lwip_ipaddr, lwip_netmask, udp_socket_ip);
    sys_thread_new("UdpSocketRecvTask", UdpSocketRecvTask, NULL, LWIP_TASK_STACK_SIZE, LWIP_DEMO_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     UDPSocketRecv, UdpSocketRecvTest, UDP Receive DEMO);

static void UdpSocketSendTask(void *arg)
{
    int cnt = LWIP_DEMO_TIMES;
    char send_str[128];
    int fd = -1;

    memset(send_str, 0, sizeof(send_str));

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd < 0)
    {
        lw_error("Socket error\n");
        return;
    }

    struct sockaddr_in udp_sock;
    udp_sock.sin_family = AF_INET;
    udp_sock.sin_port = htons(udp_socket_port);
    udp_sock.sin_addr.s_addr = PP_HTONL(LWIP_MAKEU32(udp_socket_ip[0], udp_socket_ip[1], udp_socket_ip[2], udp_socket_ip[3]));
    memset(&(udp_sock.sin_zero), 0, sizeof(udp_sock.sin_zero));

    if(connect(fd, (struct sockaddr *)&udp_sock, sizeof(struct sockaddr)))
    {
        lw_error("Unable to connect\n");
        closesocket(fd);
        return;
    }

    lw_print("UDP connect success, start to send.\n");
    lw_notice("\n\nTarget Port:%d\n\n", udp_sock.sin_port);

    while (cnt --)
    {
        snprintf(send_str, sizeof(send_str), "UDP test package times %d\r\n", cnt);
        sendto(fd, send_str, strlen(send_str), 0, (struct sockaddr*)&udp_sock, sizeof(struct sockaddr));
        lw_notice("Send UDP msg: %s ", send_str);
        MdelayKTask(1000);
    }

    closesocket(fd);
    return;
}

void UdpSocketSendTest(int argc, char *argv[])
{
    if(argc >= 2)
    {
        lw_notice("lw: [%s] target ip %s\n", __func__, argv[1]);
        if(sscanf(argv[1], "%d.%d.%d.%d:%d", &udp_socket_ip[0], &udp_socket_ip[1], &udp_socket_ip[2], &udp_socket_ip[3], &udp_socket_port) == EOK)
        {
            sscanf(argv[1], "%d.%d.%d.%d", &udp_socket_ip[0], &udp_socket_ip[1], &udp_socket_ip[2], &udp_socket_ip[3]);
        }
    }

    lwip_config_tcp(lwip_ipaddr, lwip_netmask, udp_socket_ip);
    sys_thread_new("UdpSocketSendTask", UdpSocketSendTask, NULL, LWIP_TASK_STACK_SIZE, LWIP_DEMO_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     UDPSocketSend, UdpSocketSendTest, UDP send echo);

