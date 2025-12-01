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
 * @file sx1278.c
 * @brief Implement the connection lora adapter function
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.10.20
 */

#include <adapter.h>

#ifdef ADD_NUTTX_FETURES
 /**
 * @description: Sx127x_Nuttx_Write function for nuttx
 * @param fd - file descriptor to write to
 * @param buf - Data to write
 * @param buf - Length of data to write
 * @return On success, the number of bytes written are returned (zero indicates nothing was written). On error, -1 is returned.
 */
static int Sx127x_Nuttx_Write(int fd, const void *buf, size_t len)
{
    int ret;

    unsigned char *buffer = (unsigned char*)PrivMalloc(256);
    if (!buffer)
    {
      printf("failed to allocate buffer\n");
    }
    memset(buffer, 0, 256);
    memcpy(buffer,(unsigned char *)buf,len);

    ret = PrivWrite(fd, buffer, len);
    PrivFree(buffer);

    return ret;
}
#endif
/**
 * @description: Open SX1278 spi function
 * @param adapter - Lora device pointer
 * @return success: 0, failure: -1
 */
static int Sx1278Open(struct Adapter *adapter)
{
    /*step1: open sx1278 spi port*/
    adapter->fd = PrivOpen(ADAPTER_SX1278_DRIVER, O_RDWR);
    if (adapter->fd < 0) {
        printf("Sx1278Open get spi %s fd error\n", ADAPTER_SX1278_DRIVER);
        return -1;
    }

    printf("Sx1278Open done\n");

    return 0;
}

/**
 * @description: Close SX1278 spi function
 * @param adapter - Lora device pointer
 * @return success: 0, failure: -1
 */
static int Sx1278Close(struct Adapter *adapter)
{
    /*step1: close sx1278 spi port*/
    int ret;
    ret = PrivClose(adapter->fd);
    if(ret < 0){
        printf("Sx1278 close failed: %d!\n", ret);
        return -1;
    }

    ADAPTER_DEBUG("Sx1278 Close done\n");

    return 0;
}

/**
 * @description: SX1278 ioctl function
 * @param adapter - Lora device pointer
 * @param cmd - ioctl cmd
 * @param args - iotl params
 * @return success: 0, failure: -1
 */
static int Sx1278Ioctl(struct Adapter *adapter, int cmd, void *args)
{
    /*to do*/
    return 0;
}

/**
 * @description: SX1278 join lora net group function
 * @param adapter - Lora device pointer
 * @param priv_net_group - priv_net_group params
 * @return success: 0, failure: -1
 */
#ifdef ADD_NUTTX_FETURES
static int Sx1278Join(struct Adapter *adapter, unsigned char *priv_net_group)
{
    int ret;
    struct AdapterData *priv_net_group_data = (struct AdapterData *)priv_net_group;

    ret = Sx127x_Nuttx_Write(adapter->fd, (void *)priv_net_group_data->buffer, priv_net_group_data->len);
    if(ret < 0){
        printf("Sx1278 Join net group failed: %d!\n", ret);
    }

    return ret;
}
#else
static int Sx1278Join(struct Adapter *adapter, unsigned char *priv_net_group)
{
    int ret;
    struct AdapterData *priv_net_group_data = (struct AdapterData *)priv_net_group;

    ret = PrivWrite(adapter->fd, (void *)priv_net_group_data->buffer, priv_net_group_data->len);
    if(ret < 0){
        printf("Sx1278 Join net group failed: %d!\n", ret);
    }

    return ret;
}
#endif


/**
 * @description: SX1278 send data function
 * @param adapter - Lora device pointer
 * @param buf - data buffers
 * @param len - data len
 * @return success: 0, failure: -1
 */
#ifdef ADD_NUTTX_FETURES
static int Sx1278Send(struct Adapter *adapter, const void *buf, size_t len)
{
    int ret;
    ret = Sx127x_Nuttx_Write(adapter->fd, buf, len);
    if(ret < 0){
        printf("send failed %d!\n", ret);
    }
    return ret;
}
#else
static int Sx1278Send(struct Adapter *adapter, const void *buf, size_t len)
{
    int ret;
    ret = PrivWrite(adapter->fd, buf, len);
    if(ret < 0){
        printf("send failed %d!\n", ret);
    }
    
    return ret;
}
#endif

/**
 * @description: SX1278 receive data function 
 * @param adapter - Lora device pointer
 * @param buf - data buffers
 * @param len - data len
 * @return success: 0, failure: -1
 */
#ifdef ADD_NUTTX_FETURES
static int Sx1278Recv(struct Adapter *adapter, void *buf, size_t len)
{
    int ret;
    struct sx127x_read_hdr_s recv_data;
    ret = read(adapter->fd, &recv_data, sizeof(struct sx127x_read_hdr_s));
    if (ret <= 0){
        printf("Read failed %d!\n", ret);
        return ret;
    }
    memcpy((uint8 *)buf, (uint8 *)(&recv_data), len);
    return ret;
}
#else
static int Sx1278Recv(struct Adapter *adapter, void *buf, size_t len)
{
    return PrivRead(adapter->fd, buf, len);
}
#endif

/**
 * @description: SX1278 quit lora net group function
 * @param adapter - Lora device pointer
 * @param priv_net_group - priv_net_group params
 * @return success: 0, failure: -1
 */
#ifdef ADD_NUTTX_FETURES
static int Sx1278Quit(struct Adapter *adapter, unsigned char *priv_net_group)
{
    int ret;
    struct AdapterData *priv_net_group_data = (struct AdapterData *)priv_net_group;

    ret = Sx127x_Nuttx_Write(adapter->fd, (void *)priv_net_group_data->buffer, priv_net_group_data->len);
    if(ret < 0){
        printf("Sx1278 quit net group failed %d!\n", ret);
    }

    return ret;
}
#else
static int Sx1278Quit(struct Adapter *adapter, unsigned char *priv_net_group)
{
    int ret;
    struct AdapterData *priv_net_group_data = (struct AdapterData *)priv_net_group;

    ret = PrivWrite(adapter->fd, (void *)priv_net_group_data->buffer, priv_net_group_data->len);
    if(ret < 0){
        printf("Sx1278 quit net group failed %d!\n", ret);
    }

    return ret;
}

#endif

static const struct PrivProtocolDone sx1278_done = 
{
    .open = Sx1278Open,
    .close = Sx1278Close,
    .ioctl = Sx1278Ioctl,
    .setup = NULL,
    .setdown = NULL,
    .setaddr = NULL,
    .setdns = NULL,
    .setdhcp = NULL,
    .ping = NULL,
    .netstat = NULL,
    .join = Sx1278Join,
    .send = Sx1278Send,
    .recv = Sx1278Recv,
    .quit = Sx1278Quit,
};

AdapterProductInfoType Sx1278Attach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = malloc(sizeof(struct AdapterProductInfo));
    if (!product_info) {
        printf("Sx1278Attach malloc product_info error\n");
        return NULL;
    }

    strncpy(product_info->model_name, ADAPTER_LORA_SX1278,sizeof(product_info->model_name));
    product_info->model_done = (void *)&sx1278_done;

    return product_info;
}
