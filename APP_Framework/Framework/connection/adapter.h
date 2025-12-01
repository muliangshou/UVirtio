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
 * @file adapter.h
 * @brief Structure and function declarations of the connection adapter framework
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.05.10
 */

#ifndef ADAPTER_H
#define ADAPTER_H

#include <list.h>
#include <transform.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <at_agent.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADAPTER_BUFFSIZE 64

#define ADAPTER_AT_OPERATION    1
#define ADAPTER_LWIP_OPERATION    2
#define ADAPTER_RAWIP_OPERATION    3

#ifdef CONNECTION_FRAMEWORK_DEBUG
#define ADAPTER_DEBUG printf
#else
#define ADAPTER_DEBUG
#endif

#define ADAPTER_4G_NAME "4G"
#define ADAPTER_BLUETOOTH_NAME "bluetooth"
#define ADAPTER_ETHERNET_NAME "ethernet"
#define ADAPTER_ETHERCAT_NAME "ethercat"
#define ADAPTER_LORA_NAME "lora"
#define ADAPTER_NBIOT_NAME "nbiot"
#define ADAPTER_WIFI_NAME "wifi"
#define ADAPTER_ZIGBEE_NAME "zigbee"

struct Adapter;
struct AdapterProductInfo;
typedef struct Adapter *AdapterType;
typedef struct AdapterProductInfo *AdapterProductInfoType;

struct Socket
{
    uint8_t type;                ///< socket type:DGRAM->UDP,STREAM->TCP
    uint8_t protocal;            ///< udp or tcp
    unsigned short listen_port;  ///< 0-65535
    uint8_t socket_id;           ///< socket id
    uint8_t recv_control;        ///< receive control
    uint8_t af_type;             ///< IPv4 or IPv6
    char *src_ip_addr;           ///< source P address  
    char *dst_ip_addr;           ///< destination IP address
};

enum AdapterType
{
    ADAPTER_TYPE_LORA = 0,
    ADAPTER_TYPE_4G ,
    ADAPTER_TYPE_NBIOT ,
    ADAPTER_TYPE_WIFI ,
    ADAPTER_TYPE_ETHERNET ,
    ADAPTER_TYPE_BLUETOOTH ,
    ADAPTER_TYPE_ZIGBEE ,
    ADAPTER_TYPE_5G ,
};

enum NetProtocolType
{
    PRIVATE_PROTOCOL = 1,
    IP_PROTOCOL,
    PROTOCOL_NONE,
};

enum NetRoleType
{
    CLIENT = 1, 
    SERVER,
    MASTER,
    SLAVE,
    COORDINATOR,
    ROUTER,
    END_DEVICE,
    GATEWAY,
    ROLE_NONE,
};

enum AdapterStatus
{
    REGISTERED = 1,
    UNREGISTERED,
    INSTALL,
    UNINSTALL,
};

enum IpType
{
    IPV4 = 1,
    IPV6,
};

struct AdapterData
{
    uint32 len;
    uint8 *buffer;
};

struct AdapterProductInfo
{
    uint32_t functions;
    char vendor_name[NAME_NUM_MAX];
    char model_name[NAME_NUM_MAX];
    uint32_t work_mode;

    void *model_done;
};

struct IpProtocolDone
{
    int (*open)(struct Adapter *adapter);
    int (*close)(struct Adapter *adapter);
    int (*ioctl)(struct Adapter *adapter, int cmd, void *args);
    int (*setup)(struct Adapter *adapter);
    int (*setdown)(struct Adapter *adapter);
    int (*setaddr)(struct Adapter *adapter, const char *ip, const char *gateway, const char *netmask);
    int (*setdns)(struct Adapter *adapter, const char *dns_addr, uint8 dns_count);
    int (*setdhcp)(struct Adapter *adapter, int enable);
    int (*ping)(struct Adapter *adapter, const char *destination);
    int (*netstat)(struct Adapter *adapter);
    int (*connect)(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type);
    int (*send)(struct Adapter *adapter, const void *buf, size_t len);
    int (*recv)(struct Adapter *adapter, void *buf, size_t len);
    int (*disconnect)(struct Adapter *adapter);
};

struct PrivProtocolDone
{
    int (*open)(struct Adapter *adapter);
    int (*close)(struct Adapter *adapter);
    int (*ioctl)(struct Adapter *adapter, int cmd, void *args);
    int (*setup)(struct Adapter *adapter);
    int (*setdown)(struct Adapter *adapter);
    int (*setaddr)(struct Adapter *adapter, const char *ip, const char *gateway, const char *netmask);
    int (*setdns)(struct Adapter *adapter, const char *dns_addr, uint8 dns_count);
    int (*setdhcp)(struct Adapter *adapter, int enable);
    int (*ping)(struct Adapter *adapter, const char *destination);
    int (*netstat)(struct Adapter *adapter);
    int (*join)(struct Adapter *adapter, unsigned char *priv_net_group);
    int (*send)(struct Adapter *adapter, const void *buf, size_t len);
    int (*recv)(struct Adapter *adapter, void *buf, size_t len);
    int (*quit)(struct Adapter *adapter, unsigned char *priv_net_group);
};

struct Adapter
{
    char name[NAME_NUM_MAX];
    int fd;

    int product_info_flag;
    struct AdapterProductInfo *info;
    ATAgentType agent;

    struct Socket socket;

    int net_role_id;
    enum NetProtocolType net_protocol;
    enum NetRoleType net_role;
    enum AdapterStatus adapter_status;

    char buffer[ADAPTER_BUFFSIZE];
    
    void *done;
    void *adapter_param;

    struct DoublelistNode link;
};

/*Init adapter framework*/
int AdapterFrameworkInit(void);

/*Find adapter device by name*/
AdapterType AdapterDeviceFindByName(const char *name);

/*Register the adapter to the linked list*/
int AdapterDeviceRegister(struct Adapter *adapter);

/*Unregister the adapter from the linked list*/
int AdapterDeviceUnregister(struct Adapter *adapter);

/*Open adapter device*/
int AdapterDeviceOpen(struct Adapter *adapter);

/*Close adapter device*/
int AdapterDeviceClose(struct Adapter *adapter);

/*Receice data from adapter*/
ssize_t AdapterDeviceRecv(struct Adapter *adapter, void *dst, size_t len);

/*Send data to adapter*/
ssize_t AdapterDeviceSend(struct Adapter *adapter, const void *src, size_t len);

/*Configure adapter device*/
int AdapterDeviceControl(struct Adapter *adapter, int cmd, void *args);

/*Connect to a certain ip net, only support IP_PROTOCOL*/
int AdapterDeviceConnect(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type);

/*Join to a certain private net, only support PRIVATE_PROTOCOL*/
int AdapterDeviceJoin(struct Adapter *adapter, unsigned char *priv_net_group);

/*Adapter disconnect from ip net or private net group*/
int AdapterDeviceDisconnect(struct Adapter *adapter, unsigned char *priv_net_group);

/*Set up to net*/
int AdapterDeviceSetUp(struct Adapter *adapter);

/*Set down from net*/
int AdapterDeviceSetDown(struct Adapter *adapter);

/*Set ip/gateway/netmask address*/
int AdapterDeviceSetAddr(struct Adapter *adapter, const char *ip, const char *gateway, const char *netmask);

/*Set DNS function*/
int AdapterDeviceSetDns(struct Adapter *adapter, const char *dns_addr, uint8 dns_count);

/*Set DHCP function*/
int AdapterDeviceSetDhcp(struct Adapter *adapter, int enable);

/*ping function*/
int AdapterDevicePing(struct Adapter *adapter, const char *destination);

/*Show the net status*/
int AdapterDeviceNetstat(struct Adapter *adapter);

#ifdef __cplusplus
}
#endif

#endif