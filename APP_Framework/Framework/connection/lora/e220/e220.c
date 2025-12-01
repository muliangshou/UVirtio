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
 * @file e220.c
 * @brief Implement the connection E220-400T22S lora adapter function
 * @version 2.0
 * @author AIIT XUOS Lab
 * @date 2022.4.20
 */

#include <adapter.h>

#define E220_GATEWAY_ADDRESS 0xFFFF
#define E220_CHANNEL 0x04

#ifdef AS_LORA_GATEWAY_ROLE
#define E220_ADDRESS E220_GATEWAY_ADDRESS
#endif

#ifdef AS_LORA_CLIENT_ROLE
#define E220_ADDRESS 0xFFFF
#endif

#define E220_UART_BAUD_RATE 9600

enum E220LoraMode
{
    DATA_TRANSFER_MODE = 0,     //M1 : M0 = 0 : 0
    WOR_SEND_MODE,              //M1 : M0 = 0 : 1
    WOR_RECEIVE_MODE,           //M1 : M0 = 1 : 0
    CONFIGURE_MODE_MODE,        //M1 : M0 = 1 : 1
};

/**
 * @description: Config E220 work mode by set M1/M0 pin
 * @param mode Lora working mode
 * @return NULL
 */
static void E220LoraModeConfig(enum E220LoraMode mode)
{
    //delay 1s , wait AUX ready
    PrivTaskDelay(1000);
    
    int pin_fd;
    pin_fd = PrivOpen(ADAPTER_E220_PIN_DRIVER, O_RDWR);
    if (pin_fd < 0) {
        printf("open %s error\n", ADAPTER_E220_PIN_DRIVER);
        return;
    }

    //Step1: config M0 and M1 GPIO
    struct PinParam pin_param;
    pin_param.cmd = GPIO_CONFIG_MODE;
    pin_param.mode = GPIO_CFG_OUTPUT;
    pin_param.pin = ADAPTER_E220_M0;

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = PIN_TYPE;
    ioctl_cfg.args = &pin_param;
    PrivIoctl(pin_fd, OPE_CFG, &ioctl_cfg);

    pin_param.pin = ADAPTER_E220_M1;
    ioctl_cfg.args = &pin_param;
    PrivIoctl(pin_fd, OPE_CFG, &ioctl_cfg);

    //Step2 : set M0 and M1 high or low
    struct PinStat pin_stat;

    switch (mode)
    {
    case DATA_TRANSFER_MODE:
        pin_stat.pin = ADAPTER_E220_M1;
        pin_stat.val = GPIO_LOW;
        PrivWrite(pin_fd, &pin_stat, 1);

        pin_stat.pin = ADAPTER_E220_M0;
        pin_stat.val = GPIO_LOW;
        PrivWrite(pin_fd, &pin_stat, 1);
        break;

    case WOR_SEND_MODE:
        pin_stat.pin = ADAPTER_E220_M1;
        pin_stat.val = GPIO_LOW;
        PrivWrite(pin_fd, &pin_stat, 1);

        pin_stat.pin = ADAPTER_E220_M0;
        pin_stat.val = GPIO_HIGH;
        PrivWrite(pin_fd, &pin_stat, 1);
        break;
    
    case WOR_RECEIVE_MODE:
        pin_stat.pin = ADAPTER_E220_M1;
        pin_stat.val = GPIO_HIGH;
        PrivWrite(pin_fd, &pin_stat, 1);

        pin_stat.pin = ADAPTER_E220_M0;
        pin_stat.val = GPIO_LOW;
        PrivWrite(pin_fd, &pin_stat, 1);
        break;

    case CONFIGURE_MODE_MODE:
        pin_stat.pin = ADAPTER_E220_M1;
        pin_stat.val = GPIO_HIGH;
        PrivWrite(pin_fd, &pin_stat, 1);

        pin_stat.pin = ADAPTER_E220_M0;
        pin_stat.val = GPIO_HIGH;
        PrivWrite(pin_fd, &pin_stat, 1);
        break;
    default:
        break;
    }

    PrivClose(pin_fd);

    //delay 20ms , wait mode switch done
    PrivTaskDelay(20);
}

/**
 * @description: Switch baud rate to register bit
 * @param baud_rate - baud_rate
 * @return baud_rate_bit
 */
static uint8 E220BaudRateSwitch(uint32 baud_rate)
{
    uint8 baud_rate_bit;

    switch (baud_rate)
    {
    case 1200:
        baud_rate_bit = 0x0;
        break;

    case 2400:
        baud_rate_bit = 0x1;
        break;

    case 4800:
        baud_rate_bit = 0x2;
        break;

    case 9600:
        baud_rate_bit = 0x3;
        break;

    case 19200:
        baud_rate_bit = 0x4;
        break;

    case 38400:
        baud_rate_bit = 0x5;
        break;

    case 57600:
        baud_rate_bit = 0x6;
        break;

    case 115200:
        baud_rate_bit = 0x7;
        break;
    
    default:
        break;
    }

    return baud_rate_bit;
}

/**
 * @description: Set E220 register, such as address、channel、baud rate...
 * @param adapter - lora adapter
 * @param address - address
 * @param channel - channel
 * @param baud_rate - baud_rate
 * @return success: 0, failure: -1
 */
static int E220SetRegisterParam(struct Adapter *adapter, uint16 address, uint8 channel, uint32 baud_rate)
{
    int ret;
    uint8 buffer[50] = {0};
    uint8 baud_rate_bit = E220BaudRateSwitch(baud_rate);

    E220LoraModeConfig(CONFIGURE_MODE_MODE);
    PrivTaskDelay(30);

    buffer[0] = 0xC0;                   //write register order
    buffer[1] = 0x00;                   //register start-address
    buffer[2] = 0x08;                   //register length

    buffer[3] = (address >> 8) & 0xFF;  //high address
    buffer[4] = address & 0xFF;         //low adderss

    buffer[5] = ((baud_rate_bit << 5) & 0xE0) | 0x04;

    buffer[6] = 0x00;
    buffer[7] = channel;                 //channel
    buffer[8] = 0x03;
    buffer[9] = 0;                       //high-cipher
    buffer[10] = 0;                      //low-cipher

    ret = PrivWrite(adapter->fd, (void *)buffer, 11);
    if(ret < 0){
        printf("E220SetRegisterParam send failed %d!\n", ret);
    }

    PrivRead(adapter->fd, buffer, 11);

    E220LoraModeConfig(DATA_TRANSFER_MODE);
    PrivTaskDelay(1000);

    return 0;
}        

/**
 * @description: Get E220 register, such as address、channel、baud rate...
 * @param buf - data buf
 * @return success: 0, failure: -1
 */
static int E220GetRegisterParam(uint8 *buf)
{
    int ret;
    uint8 buffer[3] = {0};

    struct Adapter *adapter = AdapterDeviceFindByName(ADAPTER_LORA_NAME);
    if (NULL == adapter) {
        printf("E220GetRegisterParam find lora adapter error\n");
        return -1;
    }

    E220LoraModeConfig(CONFIGURE_MODE_MODE);
    PrivTaskDelay(30);

    buffer[0] = 0xC1;                   //read register order
    buffer[1] = 0x00;                   //register start-address
    buffer[2] = 0x08;                   //register length

    ret = PrivWrite(adapter->fd, (void *)buffer, 3);
    if(ret < 0){
        printf("E220GetRegisterParam send failed %d!\n", ret);
    }

    PrivRead(adapter->fd, buf, 11);

    E220LoraModeConfig(DATA_TRANSFER_MODE);
    PrivTaskDelay(30);

    return 0;
}  

/**
 * @description: Open E220 uart function
 * @param adapter - Lora device pointer
 * @return success: 0, failure: -1
 */
static int E220Open(struct Adapter *adapter)
{
    /*step1: open e220 uart port*/
    adapter->fd = PrivOpen(ADAPTER_E220_DRIVER, O_RDWR);
    if (adapter->fd < 0) {
        printf("E220Open get uart %s fd error\n", ADAPTER_E220_DRIVER);
        return -1;
    }

    struct SerialDataCfg cfg;
    memset(&cfg, 0 ,sizeof(struct SerialDataCfg));

    cfg.serial_baud_rate = BAUD_RATE_9600;
    cfg.serial_data_bits = DATA_BITS_8;
    cfg.serial_stop_bits = STOP_BITS_1;
    cfg.serial_parity_mode = PARITY_NONE;
    cfg.serial_bit_order = BIT_ORDER_LSB;
    cfg.serial_invert_mode = NRZ_NORMAL;
    cfg.serial_buffer_size = SERIAL_RB_BUFSZ;

    /*aiit board use ch438, so it needs more serial configuration*/
#ifdef ADAPTER_E220_DRIVER_EXTUART
    cfg.ext_uart_no         = ADAPTER_E220_DRIVER_EXT_PORT;
    cfg.port_configure      = PORT_CFG_INIT;
#endif

    struct PrivIoctlCfg ioctl_cfg;
    ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
    ioctl_cfg.args = &cfg;

    PrivIoctl(adapter->fd, OPE_INT, &ioctl_cfg);

    E220SetRegisterParam(adapter, E220_ADDRESS, E220_CHANNEL, E220_UART_BAUD_RATE);

    ADAPTER_DEBUG("E220Open done\n");

    return 0;
}

/**
 * @description: Close E220 uart function
 * @param adapter - Lora device pointer
 * @return success: 0, failure: -1
 */
static int E220Close(struct Adapter *adapter)
{
    /*step1: close e220 uart port*/
    int ret;
    ret = PrivClose(adapter->fd);
    if(ret < 0) {
        printf("E220 close failed: %d!\n", ret);
        return -1;
    }

    ADAPTER_DEBUG("E220 Close done\n");

    return 0;
}

/**
 * @description: E220 ioctl function
 * @param adapter - Lora device pointer
 * @param cmd - ioctl cmd
 * @param args - iotl params
 * @return success: 0, failure: -1
 */
static int E220Ioctl(struct Adapter *adapter, int cmd, void *args)
{
    /*to do*/
    return 0;
}

/**
 * @description: E220 join lora net group function
 * @param adapter - Lora device pointer
 * @param priv_net_group - priv_net_group params
 * @return success: 0, failure: -1
 */
static int E220Join(struct Adapter *adapter, unsigned char *priv_net_group)
{
    int ret;
    struct AdapterData *priv_net_group_data = (struct AdapterData *)priv_net_group;

    ret = PrivWrite(adapter->fd, (void *)priv_net_group_data->buffer, priv_net_group_data->len);
    if(ret < 0) {
        printf("E220 Join net group failed: %d!\n", ret);
    }

    return ret;
}

/**
 * @description: E220 send data function
 * @param adapter - Lora device pointer
 * @param buf - data buffers
 * @param len - data len
 * @return success: 0, failure: -1
 */
static int E220Send(struct Adapter *adapter, const void *buf, size_t len)
{
    int ret;

    ret = PrivWrite(adapter->fd, (void *)buf, len);
    if(ret < 0){
        printf("send failed %d!\n", ret);
    }
    
    return ret;
}

/**
 * @description: E220 receive data function 
 * @param adapter - Lora device pointer
 * @param buf - data buffers
 * @param len - data len
 * @return success: 0, failure: -1
 */
static int E220Recv(struct Adapter *adapter, void *buf, size_t len)
{
    int recv_len, recv_len_continue;

    uint8 *recv_buf = PrivMalloc(len);

    recv_len = PrivRead(adapter->fd, recv_buf, len);
    while (recv_len < len) {
        recv_len_continue = PrivRead(adapter->fd, recv_buf + recv_len, len - recv_len);

        recv_len += recv_len_continue;
    }

    memcpy(buf, recv_buf, recv_len);

    PrivFree(recv_buf);
    
    return recv_len;
}

/**
 * @description: E220 quit lora net group function
 * @param adapter - Lora device pointer
 * @param priv_net_group - priv_net_group params
 * @return success: 0, failure: -1
 */
static int E220Quit(struct Adapter *adapter, unsigned char *priv_net_group)
{
    int ret;

    struct AdapterData *priv_net_group_data = (struct AdapterData *)priv_net_group;

    ret = PrivWrite(adapter->fd, (void *)priv_net_group_data->buffer, priv_net_group_data->len);
    if(ret < 0){
        printf("E220 quit net group failed %d!\n", ret);
    }

    return ret;
}

static const struct PrivProtocolDone e220_done = 
{
    .open = E220Open,
    .close = E220Close,
    .ioctl = E220Ioctl,
    .setup = NULL,
    .setdown = NULL,
    .setaddr = NULL,
    .setdns = NULL,
    .setdhcp = NULL,
    .ping = NULL,
    .netstat = NULL,
    .join = E220Join,
    .send = E220Send,
    .recv = E220Recv,
    .quit = E220Quit,
};

AdapterProductInfoType E220Attach(struct Adapter *adapter)
{
    struct AdapterProductInfo *product_info = malloc(sizeof(struct AdapterProductInfo));
    if (!product_info) {
        printf("E220Attach malloc product_info error\n");
        return NULL;
    }

    strncpy(product_info->model_name, ADAPTER_LORA_E220,sizeof(product_info->model_name));
    product_info->model_done = (void *)&e220_done;

    return product_info;
}

//###################TEST####################
static void LoraOpen(void)
{
    struct Adapter *adapter = AdapterDeviceFindByName(ADAPTER_LORA_NAME);
    if (NULL == adapter) {
        printf("LoraReceive find lora adapter error\n");
        return;
    }

    E220Open(adapter);
}

static void LoraRead(void *parameter)
{
	int RevLen;
	uint8 i, cnt = 0;

    uint8 buffer[256];

    memset(buffer, 0, 256);

    struct Adapter *adapter = AdapterDeviceFindByName(ADAPTER_LORA_NAME);
    if (NULL == adapter) {
        printf("LoraRead find lora adapter error\n");
        return;
    }
	
    while (1)
    {
        KPrintf("ready to read lora data\n");

        RevLen = E220Recv(adapter, buffer, 256);
		if (RevLen) {
            KPrintf("lora get data %u\n", RevLen);
            for (i = 0; i < RevLen; i ++) {
                printf("i %u data 0x%x\n", i, buffer[i]);
            }

            memset(buffer, 0, 256);

            PrivTaskDelay(1000);

			cnt ++;
            E220Send(adapter, &cnt, 1);
		}
	}
}

static void LoraTest(void)
{
    int ret;

    LoraOpen();

	int task_lora_read = KTaskCreate("task_lora_read", LoraRead, NONE, 2048, 10); 
	ret = StartupKTask(task_lora_read);
    if (ret != EOK) {
		KPrintf("StartupKTask task_lora_read failed .\n");
		return;
	} 
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
LoraTest, LoraTest, lora send and receive message);

static void LoraSend(int argc, char *argv[])
{
    struct Adapter *adapter = AdapterDeviceFindByName(ADAPTER_LORA_NAME);
    if (NULL == adapter) {
        printf("LoraRead find lora adapter error\n");
        return;
    }

    char Msg[256] = {0};

    if (argc == 2) {
        strncpy(Msg, argv[1], 256);

        E220Send(adapter, Msg, strlen(Msg));
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
LoraSend, LoraSend, lora send message);
