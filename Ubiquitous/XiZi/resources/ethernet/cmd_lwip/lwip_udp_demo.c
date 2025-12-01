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
* @file lwip_udp_demo.c
* @brief One UDP demo based on LwIP
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-21
*/
#include "board.h"
#include "sys_arch.h"
#include "lwip/udp.h"
#include "lwip/sockets.h"


#define PBUF_SIZE 27

static struct udp_pcb *udpecho_raw_pcb;

char udp_demo_ip[] = {192, 168, 250, 252};
u16_t udp_demo_port = LWIP_TARGET_PORT;

char hello_str[] = {"hello world\r\n"};
char udp_demo_msg[] = "\nThis one is UDP package!!!\n";

/******************************************************************************/

static void LwipUDPSendTask(void *arg)
{
    int cnt = LWIP_DEMO_TIMES;

    lw_print("udp_send_demo start.\n");

    int socket_fd = -1;
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        lw_error("Socket error\n");
        return;
    }

    struct sockaddr_in udp_sock;
    udp_sock.sin_family = AF_INET;
    udp_sock.sin_port = htons(udp_demo_port);
    udp_sock.sin_addr.s_addr = PP_HTONL(LWIP_MAKEU32(udp_demo_ip[0], udp_demo_ip[1], udp_demo_ip[2], udp_demo_ip[3]));
    memset(&(udp_sock.sin_zero), 0, sizeof(udp_sock.sin_zero));

    if (connect(socket_fd, (struct sockaddr *)&udp_sock, sizeof(struct sockaddr)))
    {
        lw_error("Unable to connect\n");
        closesocket(socket_fd);
        return;
    }

    lw_notice("UDP connect success, start to send.\n");
    lw_notice("\n\nTarget Port:%d\n\n", udp_sock.sin_port);

    sendto(socket_fd, udp_demo_msg, strlen(udp_demo_msg), 0, (struct sockaddr*)&udp_sock, sizeof(struct sockaddr));
    lw_notice("Send UDP msg: %s ", udp_demo_msg);
    closesocket(socket_fd);
    return;
}

void *LwipUdpSendTest(int argc, char *argv[])
{
    memset(udp_demo_msg, 0, sizeof(udp_demo_msg));

    if(argc == 1)
    {
        lw_print("lw: [%s] gw %d.%d.%d.%d\n", __func__, udp_demo_ip[0], udp_demo_ip[1], udp_demo_ip[2], udp_demo_ip[3]);
        strncpy(udp_demo_msg, hello_str, strlen(hello_str));
    }
    else
    {
        strncpy(udp_demo_msg, argv[1], strlen(argv[1]));
        strncat(udp_demo_msg, "\r\n", 2);
        if(argc == 3)
        {
            sscanf(argv[2], "%d.%d.%d.%d", &udp_demo_ip[0], &udp_demo_ip[1], &udp_demo_ip[2], &udp_demo_ip[3]);
        }
    }

    lw_print("lw: [%s] gw %d.%d.%d.%d\n", __func__, udp_demo_ip[0], udp_demo_ip[1], udp_demo_ip[2], udp_demo_ip[3]);

    lwip_config_net(lwip_ipaddr, lwip_netmask, udp_demo_ip);
    sys_thread_new("udp send", LwipUDPSendTask, NULL, LWIP_TASK_STACK_SIZE, LWIP_DEMO_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     UDPSend, LwipUdpSendTest, UDP send echo);

static void LwipUdpRecvTask(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                 const ip_addr_t *addr, u16_t port)
{
    int udp_len;
    err_t err;
    struct pbuf* udp_buf;

    LWIP_UNUSED_ARG(arg);

    if (p == NULL)
    {
        return;
    }

    udp_len = p->tot_len;
    lw_notice("Receive data :%dB\r\n", udp_len);

    if(udp_len <= 80)
    {
        lw_notice("%.*s\r\n", udp_len, (char *)(p->payload));
    }

    udp_buf = pbuf_alloc(PBUF_TRANSPORT, PBUF_SIZE, PBUF_RAM);

    memset(udp_buf->payload, 0, PBUF_SIZE);

    err = pbuf_take(udp_buf, "Client receive success!\r\n", 27);

    /* send received packet back to sender */
    udp_sendto(upcb, udp_buf, addr, port);

    /* free the pbuf */
    pbuf_free(p);
    pbuf_free(udp_buf);
}

void LwipUdpRecvTest(void)
{
    err_t err;

    lwip_config_net(lwip_ipaddr, lwip_netmask, lwip_gwaddr);

    udpecho_raw_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udpecho_raw_pcb == NULL)
    {
        return;
    }

    err = udp_bind(udpecho_raw_pcb, IP_ANY_TYPE, LWIP_LOCAL_PORT);
    if (err == ERR_OK)
    {
        udp_recv(udpecho_raw_pcb, LwipUdpRecvTask, NULL);
    }
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
     UDPRecv, LwipUdpRecvTest, UDP Receive echo);

