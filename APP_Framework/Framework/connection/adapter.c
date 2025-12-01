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
 * @file adapter.c
 * @brief Implement the connection adapter framework management and API
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.05.10
 */

#include <adapter.h>

static DoublelistType adapter_list;

static int adapter_list_lock;

/**
 * @description: Init adapter framework
 * @return 0
 */
int AdapterFrameworkInit(void)
{
    int ret = 0;
    AppInitDoubleList(&adapter_list);

    ret = PrivMutexCreate(&adapter_list_lock, 0);
    if(ret != 0) {
        printf("AdapterFrameworkInit mutex create failed.\n");
    }

    return 0;
}

/**
 * @description: Find adapter device by name
 * @param name - name string
 * @return adapter device pointer
 */
AdapterType AdapterDeviceFindByName(const char *name)
{
    struct Adapter *ret = NULL;
    struct DoublelistNode *node;
    int status = 0;

    if (NULL == name)
        return NULL;

    status = PrivMutexObtain(&adapter_list_lock);
    if (status != 0){
       printf("%s:pthread_mutex_lock failed, status=%d\n",__func__,status);
    }

    DOUBLE_LIST_FOR_EACH(node, &adapter_list) {
        struct Adapter *adapter = CONTAINER_OF(node,
                struct Adapter, link);
        if (0 == strncmp(adapter->name, name, NAME_NUM_MAX)) {
            ret = adapter;
            break;
        }
        printf("PrivMutexObtain in loop\n");
    }

    status = PrivMutexAbandon(&adapter_list_lock);
    if (status != 0){
       printf("%s:pthread_mutex_unlock failed, status=%d\n",__func__,status);
    }

    return ret;
}

/**
 * @description: Register the adapter to the linked list
 * @param adapter - adapter device pointer
 * @return success: 0 , failure: -1
 */
int AdapterDeviceRegister(struct Adapter *adapter)
{
    int status = 0;
    if (NULL == adapter )
        return -1;

    if (NULL != AdapterDeviceFindByName(adapter->name)) {
        printf("%s: sensor with the same name already registered\n", __func__);
        return -1;
    }

    status = PrivMutexObtain(&adapter_list_lock);
    if (status != 0){
       printf("%s:pthread_mutex_lock failed, status=%d\n",__func__,status);
    }

    AppDoubleListInsertNodeAfter(&adapter_list, &adapter->link);
    
    status = PrivMutexAbandon(&adapter_list_lock);
    if (status != 0){
       printf("%s:pthread_mutex_unlock failed, status=%d\n",__func__,status);
    }

    adapter->adapter_status = REGISTERED;

    return 0;
}

/**
 * @description: Unregister the adapter from the linked list
 * @param adapter - adapter device pointer
 * @return 0
 */
int AdapterDeviceUnregister(struct Adapter *adapter)
{
    int status = 0;
    if (!adapter)
        return -1;
    status = PrivMutexObtain(&adapter_list_lock);
    if (status != 0){
       printf("%s:pthread_mutex_lock failed, status=%d\n",__func__,status);
    }

    AppDoubleListRmNode(&adapter->link);

    status = PrivMutexAbandon(&adapter_list_lock);
    if (status != 0){
       printf("%s:pthread_mutex_unlock failed, status=%d\n",__func__,status);
    }

    adapter->adapter_status = UNREGISTERED;

    return 0;
}

/**
 * @description: Open adapter device
 * @param name - adapter device name
 * @return success: 0 , failure: other
 */
int AdapterDeviceOpen(struct Adapter *adapter)
{
    if (!adapter)
        return -1;

    if (INSTALL == adapter->adapter_status) {
        printf("Device %s has already been opened. Just return\n", adapter->name);
        return 0;
    }

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->open)
            return 0;
        
        result = priv_done->open(adapter);
        if (0 == result) {
            printf("Device %s open success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s open failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->open)
            return 0;
        
        result = ip_done->open(adapter);
        if (0 == result) {
            printf("Device %s open success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s open failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: Close adapter device
 * @param adapter - adapter device pointer
 * @return success: 0 , failure: other
 */
int AdapterDeviceClose(struct Adapter *adapter)
{
    if (!adapter)
        return -1;

    if (UNINSTALL == adapter->adapter_status) {
        printf("Device %s has already been closed. Just return\n", adapter->name);
        return 0;
    }

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->close)
            return 0;
        
        result = priv_done->close(adapter);
        if (0 == result) {
            printf("%s successfully closed.\n", adapter->name);
            adapter->adapter_status = UNINSTALL;
        } else {
            printf("Closed %s failure.\n", adapter->name);
        }

        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->close)
            return 0;
        
        result = ip_done->close(adapter);
        if (0 == result) {
            printf("%s successfully closed.\n", adapter->name);
            adapter->adapter_status = UNINSTALL;
        } else {
            printf("Closed %s failure.\n", adapter->name);
        }
        break;

    default:
        break;
    }
    
    return result;
}

/**
 * @description: Configure adapter device
 * @param adapter - adapter device pointer
 * @param cmd - command
 * @param args - command parameter
 * @return success: 0 , failure: other
 */
int AdapterDeviceControl(struct Adapter *adapter, int cmd, void *args)
{
    if (!adapter)
        return -1;
        
    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->ioctl)
            return 0;
        
        result = priv_done->ioctl(adapter, cmd, args);
        if (0 == result) {
            printf("Device %s ioctl success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s ioctl failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->ioctl)
            return 0;
        
        result = ip_done->ioctl(adapter, cmd, args);
        if (0 == result) {
            printf("Device %s ioctl success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s ioctl failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: Receice data from adapter
 * @param adapter - adapter device pointer
 * @param dst - buffer to save data
 * @param len - buffer length
 * @return gotten data length
 */
ssize_t AdapterDeviceRecv(struct Adapter *adapter, void *dst, size_t len)
{
    if (!adapter)
        return -1;

    if (PRIVATE_PROTOCOL == adapter->net_protocol) {
        struct PrivProtocolDone *priv_done = (struct PrivProtocolDone *)adapter->done; 
        
        if (NULL == priv_done->recv)
            return -1;
    
        return priv_done->recv(adapter, dst, len);
    } else if (IP_PROTOCOL == adapter->net_protocol) {
        struct IpProtocolDone *ip_done = (struct IpProtocolDone *)adapter->done;
    
        if (NULL == ip_done->recv)
            return -1;
    
        return ip_done->recv(adapter, dst, len);
    } else {
        printf("AdapterDeviceRecv net_protocol %d not support\n", adapter->net_protocol);
        return -1;
    }
}

/**
 * @description: Send data to adapter
 * @param adapter - adapter device pointer
 * @param src - data buffer
 * @param len - data length
 * @return length of data written
 */
ssize_t AdapterDeviceSend(struct Adapter *adapter, const void *src, size_t len)
{
    if (!adapter)
        return -1;

    if (PRIVATE_PROTOCOL == adapter->net_protocol) {
        struct PrivProtocolDone *priv_done = (struct PrivProtocolDone *)adapter->done; 
        
        if (NULL == priv_done->send)
            return -1;
    
        return priv_done->send(adapter, src, len);
    } else if (IP_PROTOCOL == adapter->net_protocol) {
        struct IpProtocolDone *ip_done = (struct IpProtocolDone *)adapter->done;
    
        if (NULL == ip_done->send)
            return -1;
    
        return ip_done->send(adapter, src, len);
    } else {
        printf("AdapterDeviceSend net_protocol %d not support\n", adapter->net_protocol);
        return -1;
    }
}

/**
 * @description: Connect to a certain ip net, only support IP_PROTOCOL
 * @param adapter - adapter device pointer
 * @param ip - connect ip
 * @param port - connect port
 * @param ip_type - ip type, IPV4 or IPV6
 * @return success: 0 , failure: other
 */
int AdapterDeviceConnect(struct Adapter *adapter, enum NetRoleType net_role, const char *ip, const char *port, enum IpType ip_type)
{
    if (!adapter)
        return -1;
        
    if (PRIVATE_PROTOCOL == adapter->net_protocol) {
        printf("AdapterDeviceConnect not suuport private_protocol, please use join\n");
        return -1;
    } else if (IP_PROTOCOL == adapter->net_protocol) {
        struct IpProtocolDone *ip_done = (struct IpProtocolDone *)adapter->done;
    
        if (NULL == ip_done->connect)
            return -1;
    
        return ip_done->connect(adapter, net_role, ip, port, ip_type);
    } else {
        printf("AdapterDeviceConnect net_protocol %d not support\n", adapter->net_protocol);
        return -1;
    }
}

/**
 * @description: Join to a certain private net, only support PRIVATE_PROTOCOL
 * @param adapter - adapter device pointer
 * @param priv_net_group - private net group
 * @return success: 0 , failure: other
 */
int AdapterDeviceJoin(struct Adapter *adapter, unsigned char *priv_net_group)
{
    if (!adapter)
        return -1;
        
    if (PRIVATE_PROTOCOL == adapter->net_protocol) {
        struct PrivProtocolDone *priv_done = (struct PrivProtocolDone *)adapter->done; 
        
        if (NULL == priv_done->join)
            return -1;
    
        return priv_done->join(adapter, priv_net_group);
    } else if (IP_PROTOCOL == adapter->net_protocol) {
        printf("AdapterDeviceJoin not support ip_protocol, please use connect\n");
        return -1;
    } else {
        printf("AdapterDeviceJoin net_protocol %d not support\n", adapter->net_protocol);
        return -1;
    }
}

/**
 * @description: Adapter disconnect from ip net or private net group 
 * @param adapter - adapter device pointer
 * @param priv_net_group - private net group for PRIVATE_PROTOCOL quit function
 * @return success: 0 , failure: other
 */
int AdapterDeviceDisconnect(struct Adapter *adapter, unsigned char *priv_net_group)
{
    if (!adapter)
        return -1;
        
    if (PRIVATE_PROTOCOL == adapter->net_protocol) {
        struct PrivProtocolDone *priv_done = (struct PrivProtocolDone *)adapter->done; 
        
        if (NULL == priv_done->quit)
            return -1;
    
        return priv_done->quit(adapter, priv_net_group);
    } else if (IP_PROTOCOL == adapter->net_protocol) {
        struct IpProtocolDone *ip_done = (struct IpProtocolDone *)adapter->done;
    
        if (NULL == ip_done->disconnect)
            return -1;
    
        return ip_done->disconnect(adapter);
    } else {
        printf("AdapterDeviceDisconnect net_protocol %d not support\n", adapter->net_protocol);
        return -1;
    }
}

/**
 * @description: Set up to net 
 * @param adapter - adapter device pointer
 * @return success: 0 , failure: other
 */
int AdapterDeviceSetUp(struct Adapter *adapter)
{
    if (!adapter)
        return -1;

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->setup)
            return 0;
        
        result = priv_done->setup(adapter);
        if (0 == result) {
            printf("Device %s setup success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setup failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->setup)
            return 0;
        
        result = ip_done->setup(adapter);
        if (0 == result) {
            printf("Device %s setup success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setup failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: Set down from net 
 * @param adapter - adapter device pointer
 * @return success: 0 , failure: other
 */
int AdapterDeviceSetDown(struct Adapter *adapter)
{
    if (!adapter)
        return -1;

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->setdown)
            return 0;
        
        result = priv_done->setdown(adapter);
        if (0 == result) {
            printf("Device %s setdown success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setdown failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->setdown)
            return 0;
        
        result = ip_done->setdown(adapter);
        if (0 == result) {
            printf("Device %s setdown success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setdown failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: Set ip/gateway/netmask address
 * @param adapter - adapter device pointer
 * @param ip - ip address
 * @param gateway - gateway address
 * @param netmast - netmast address
 * @return success: 0 , failure: other
 */
int AdapterDeviceSetAddr(struct Adapter *adapter, const char *ip, const char *gateway, const char *netmask)
{
    if (!adapter)
        return -1;

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->setaddr)
            return 0;
        
        result = priv_done->setaddr(adapter, ip, gateway, netmask);
        if (0 == result) {
            printf("Device %s setaddr success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setaddr failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->setaddr)
            return 0;
        
        result = ip_done->setaddr(adapter, ip, gateway, netmask);
        if (0 == result) {
            printf("Device %s setaddr success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setaddr failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: Set dns function
 * @param adapter - adapter device pointer
 * @param dns_addr - dns address
 * @param dns_count - dns count
 * @return success: 0 , failure: other
 */
int AdapterDeviceSetDns(struct Adapter *adapter, const char *dns_addr, uint8 dns_count)
{
    if (!adapter)
        return -1;

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->setdns)
            return 0;
        
        result = priv_done->setdns(adapter, dns_addr, dns_count);
        if (0 == result) {
            printf("Device %s setdns success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setdns failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->setdns)
            return 0;
        
        result = ip_done->setdns(adapter, dns_addr, dns_count);
        if (0 == result) {
            printf("Device %s setdns success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setdns failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: Set DHCP function
 * @param adapter - adapter device pointer
 * @param enable - enable DHCP or not
 * @return success: 0 , failure: other
 */
int AdapterDeviceSetDhcp(struct Adapter *adapter, int enable)
{
    if (!adapter)
        return -1;

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->setdhcp)
            return 0;
        
        result = priv_done->setdhcp(adapter, enable);
        if (0 == result) {
            printf("Device %s setdhcp success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setdhcp failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->setdhcp)
            return 0;
        
        result = ip_done->setdhcp(adapter, enable);
        if (0 == result) {
            printf("Device %s setdhcp success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s setdhcp failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: ping function
 * @param adapter - adapter device pointer
 * @param destination - the destination ip address
 * @return success: 0 , failure: other
 */
int AdapterDevicePing(struct Adapter *adapter, const char *destination)
{
    if (!adapter)
        return -1;

    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->ping)
            return 0;
        
        result = priv_done->ping(adapter, destination);
        if (0 == result) {
            printf("Device %s ping success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s ping failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->ping)
            return 0;
        
        result = ip_done->ping(adapter, destination);
        if (0 == result) {
            printf("Device %s ping success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s ping failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

/**
 * @description: Show the net status
 * @param adapter - adapter device pointer
 * @return success: 0 , failure: other
 */
int AdapterDeviceNetstat(struct Adapter *adapter)
{
    if (!adapter)
        return -1;
        
    int result = 0;

    struct IpProtocolDone *ip_done = NULL;
    struct PrivProtocolDone *priv_done = NULL;

    switch (adapter->net_protocol)
    {
    case PRIVATE_PROTOCOL:
        priv_done = (struct PrivProtocolDone *)adapter->done;
        if (NULL == priv_done->netstat)
            return 0;
        
        result = priv_done->netstat(adapter);
        if (0 == result) {
            printf("Device %s netstat success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s netstat failed(%d).\n", adapter->name, result);
        }
        break;
    
    case IP_PROTOCOL:
        ip_done = (struct IpProtocolDone *)adapter->done;
        if (NULL == ip_done->netstat)
            return 0;
        
        result = ip_done->netstat(adapter);
        if (0 == result) {
            printf("Device %s netstat success.\n", adapter->name);
            adapter->adapter_status = INSTALL;
        } else {
            if (adapter->fd) {
                PrivClose(adapter->fd);
                adapter->fd = 0;
            }
            printf("Device %s netstat failed(%d).\n", adapter->name, result);
        }
        break;

    default:
        break;
    }

    return result;
}

