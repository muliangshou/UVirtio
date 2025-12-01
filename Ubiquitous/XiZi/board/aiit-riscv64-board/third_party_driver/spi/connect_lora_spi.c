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
* @file connect_lora_spi.c
* @brief support to register spi lora pointer and function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-05-17
*/

#include <connect_spi_lora.h>

static struct HardwareDev *g_spi_lora_dev;
static BusType buspin;
tRadioDriver *Radio = NONE;
void SX1276InitIo(void)
{
    struct PinParam PinCfg;
	struct PinStat PinStat;

	struct BusConfigureInfo configure_info;
	struct BusBlockWriteParam write_param;

    write_param.buffer = (void *)&PinStat;
    configure_info.configure_cmd = OPE_CFG;
    configure_info.private_data = (void *)&PinCfg;

    buspin = PinBusInitGet();  

    PinCfg.cmd = GPIO_CONFIG_MODE;
    PinCfg.pin = SX12XX_DEVICE_DO0_PIN;
    PinCfg.mode = GPIO_CFG_INPUT;
    BusDrvConfigure(buspin->owner_driver, &configure_info);

    PinCfg.cmd = GPIO_CONFIG_MODE;
    PinCfg.pin = SX12XX_DEVICE_DO1_PIN;
    PinCfg.mode = GPIO_CFG_INPUT;
    BusDrvConfigure(buspin->owner_driver, &configure_info);

    PinCfg.cmd = GPIO_CONFIG_MODE;
    PinCfg.pin = SX12XX_DEVICE_DO2_PIN;
    PinCfg.mode = GPIO_CFG_INPUT;
    BusDrvConfigure(buspin->owner_driver, &configure_info);
}

inline uint8_t SX1276ReadDio0(void)
{
    struct PinStat PinStat;

    struct BusBlockReadParam read_param;
    read_param.buffer = (void *)&PinStat;

    PinStat.pin = SX12XX_DEVICE_DO0_PIN;
    
    return BusDevReadData(buspin->owner_haldev, &read_param);
}

inline uint8_t SX1276ReadDio1(void)
{
    struct PinStat PinStat;

    struct BusBlockReadParam read_param;
    read_param.buffer = (void *)&PinStat;

    PinStat.pin = SX12XX_DEVICE_DO1_PIN;
    
    return BusDevReadData(buspin->owner_haldev, &read_param);
}

inline uint8_t SX1276ReadDio2(void)
{
    struct PinStat PinStat;

    struct BusBlockReadParam read_param;
    read_param.buffer = (void *)&PinStat;

    PinStat.pin = SX12XX_DEVICE_DO2_PIN;
    
    return BusDevReadData(buspin->owner_haldev, &read_param);
}

inline uint8_t SX1276ReadDio3(void)
{
    struct PinStat PinStat;

    struct BusBlockReadParam read_param;
    read_param.buffer = (void *)&PinStat;

    PinStat.pin = SX12XX_DEVICE_DO3_PIN;
    
    return BusDevReadData(buspin->owner_haldev, &read_param);
}

inline uint8_t SX1276ReadDio4(void)
{
    struct PinStat PinStat;

    struct BusBlockReadParam read_param;
    read_param.buffer = (void *)&PinStat;

    PinStat.pin = SX12XX_DEVICE_DO4_PIN;
    
    return BusDevReadData(buspin->owner_haldev, &read_param);
}

inline uint8_t SX1276ReadDio5(void)
{
    struct PinStat PinStat;

    struct BusBlockReadParam read_param;
    read_param.buffer = (void *)&PinStat;

    PinStat.pin = SX12XX_DEVICE_DO5_PIN;
    
    return BusDevReadData(buspin->owner_haldev, &read_param);
}

inline void SX1276WriteRxTx(uint8_t txEnable)
{
    return;
}

void SX1276SetReset(uint8_t state)
{
    struct PinParam PinCfg;
	struct PinStat PinStat;

	struct BusConfigureInfo configure_info;
	struct BusBlockWriteParam write_param;

    configure_info.configure_cmd = OPE_CFG;
    configure_info.private_data = (void *)&PinCfg;
    write_param.buffer = (void *)&PinStat;

    if (state == RADIO_RESET_ON)
    {
        PinCfg.cmd = GPIO_CONFIG_MODE;
        PinCfg.pin = SX12XX_DEVICE_RST_PIN;
        PinCfg.mode = GPIO_CFG_OUTPUT;
        BusDrvConfigure(buspin->owner_driver, &configure_info);

        PinStat.val = GPIO_LOW;
        PinStat.pin = SX12XX_DEVICE_RST_PIN;
        BusDevWriteData(buspin->owner_haldev, &write_param);
    }
    else
    {
        PinCfg.cmd = GPIO_CONFIG_MODE;
        PinCfg.pin = SX12XX_DEVICE_RST_PIN;
        PinCfg.mode = GPIO_CFG_INPUT;
        BusDrvConfigure(buspin->owner_driver, &configure_info);
    }
}

void SX1276Write(uint8_t addr, uint8_t data)
{
    SX1276WriteBuffer(addr, &data, 1);
}

void SX1276Read(uint8_t addr, uint8_t *data)
{
    SX1276ReadBuffer(addr, data, 1);
}

void SX1276WriteBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    struct BusBlockWriteParam write_param;
    uint8 write_addr = addr | 0x80;

    BusDevOpen(g_spi_lora_dev);

    write_param.buffer = (void *)&write_addr;
    write_param.size = 1;
    BusDevWriteData(g_spi_lora_dev, &write_param);

    write_param.buffer = (void *)buffer;
    write_param.size = size;
    BusDevWriteData(g_spi_lora_dev, &write_param);

    BusDevClose(g_spi_lora_dev);
}

void SX1276ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    struct BusBlockWriteParam write_param;
    struct BusBlockReadParam read_param;

    uint8 write_addr = addr & 0x7F;

    BusDevOpen(g_spi_lora_dev);

    write_param.buffer = (void *)&write_addr;
    write_param.size = 1;
    BusDevWriteData(g_spi_lora_dev, &write_param);

    read_param.buffer = (void *)buffer;
    read_param.size = size;
    BusDevReadData(g_spi_lora_dev, &read_param);

    BusDevClose(g_spi_lora_dev);
}

void SX1276WriteFifo(uint8_t *buffer, uint8_t size)
{
    SX1276WriteBuffer(0, buffer, size);
}

void SX1276ReadFifo(uint8_t *buffer, uint8_t size)
{
    SX1276ReadBuffer(0, buffer, size);
}

uint8_t SX1276_Spi_Check()
{
    uint8_t test = 0;
    KPrintf("SX1276_Spi_Check start\n");
	tLoRaSettings settings;
	SX1276Read(REG_LR_VERSION,&test);
	KPrintf("version code of the chip is %x\n",test);
	settings.RFFrequency = SX1276LoRaGetRFFrequency();
	KPrintf("SX1278 Lora parameters are :\nRFFrequency is %d\n",settings.RFFrequency);
	settings.Power = SX1276LoRaGetRFPower();
	KPrintf("RFPower is %d\n",settings.Power);
    settings.SignalBw = SX1276LoRaGetSignalBandwidth();	  
	KPrintf("SignalBw is %d\n",settings.SignalBw);
	settings.SpreadingFactor = SX1276LoRaGetSpreadingFactor();
	KPrintf("SpreadingFactor is %d\n",settings.SpreadingFactor);
    /*SPI confirm*/
    SX1276Write(REG_LR_HOPPERIOD, 0x91);  
    SX1276Read(REG_LR_HOPPERIOD, &test);
    if (test != 0x91) {
        return 0;
    }		
    return test;
}

/**
 * This function supports to write data to the lora.
 *
 * @param dev lora dev descriptor
 * @param write_param lora dev write datacfg param
 */
static uint32 SpiLoraWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(write_param);

    if (write_param->size > 256) {
        KPrintf("SpiLoraWrite ERROR:The message is too long!\n");
        return ERROR;
    } else {
        //Radio->SetTxPacket(write_param->buffer, write_param->size);    
        //while(Radio->Process() != RF_TX_DONE);
        SX1276SetTxPacket(write_param->buffer, write_param->size);    
        while(SX1276Process() != RF_TX_DONE);
        KPrintf("SpiLoraWrite success!\n");
    }

    return EOK;
}

/**
 * This function supports to read data from the lora.
 *
 * @param dev lora dev descriptor
 * @param read_param lora dev read datacfg param
 */
static uint32 SpiLoraRead(void *dev, struct BusBlockReadParam *read_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(read_param);

    int read_times = 1000;
    
    //Radio->StartRx();
    SX1276StartRx();
    KPrintf("SpiLoraRead Ready!\n");

    while (read_times) {
        if (SX1276Process() != RF_RX_DONE) {
            read_times --;
            MdelayKTask(500);
        } else {
            break;
        }
    }

    if (read_times > 0) {
        SX1276GetRxPacket(read_param->buffer, (uint16 *)&read_param->read_length);
    } else {
        read_param->read_length = 0;
    }
    
    //while(Radio->Process() != RF_RX_DONE);
    //Radio->GetRxPacket(read_param->buffer, (uint16 *)&read_param->read_length);     
    // while(SX1276Process() != RF_RX_DONE);
    // SX1276GetRxPacket(read_param->buffer, (uint16 *)&read_param->read_length);

    return read_param->read_length;
}

static uint32 SpiLoraOpen(void *dev)
{
    NULL_PARAM_CHECK(dev);

    KPrintf("SpiLoraOpen start\n");

    x_err_t ret = EOK;
    static x_bool lora_init_status = RET_FALSE;

    if (RET_TRUE == lora_init_status) {
        return EOK;
    }

    struct HardwareDev *haldev = (struct HardwareDev *)dev;

    struct SpiHardwareDevice *lora_dev = CONTAINER_OF(haldev, struct SpiHardwareDevice, haldev);
    NULL_PARAM_CHECK(lora_dev);

    SpiLoraDeviceType spi_lora_dev = CONTAINER_OF(lora_dev, struct SpiLoraDevice, lora_dev);    
    NULL_PARAM_CHECK(spi_lora_dev);

    struct Driver *spi_drv = spi_lora_dev->spi_dev->haldev.owner_bus->owner_driver;

    struct BusConfigureInfo configure_info;
    struct SpiMasterParam spi_master_param;
    spi_master_param.spi_data_bit_width = 8;
    spi_master_param.spi_work_mode = SPI_MODE_0 | SPI_MSB;
    spi_master_param.spi_maxfrequency = SPI_LORA_FREQUENCY;
    spi_master_param.spi_data_endian = 0;

    configure_info.configure_cmd = OPE_CFG;
    configure_info.private_data = (void *)&spi_master_param;
    ret = BusDrvConfigure(spi_drv, &configure_info);
    if (ret) {
        KPrintf("spi drv OPE_CFG error drv %8p cfg %8p\n", spi_drv, &spi_master_param);
        return ERROR;
    }

    configure_info.configure_cmd = OPE_INT;
    ret = BusDrvConfigure(spi_drv, &configure_info);
    if (ret) {
        KPrintf("spi drv OPE_INT error drv %8p\n", spi_drv);
        return ERROR;
    }

    SX1276Init();

    if (0x91 != SX1276_Spi_Check()) {
        KPrintf("LoRa check failed!\n!");
    } else {
        Radio = RadioDriverInit();
        KPrintf("LoRa check ok!\nNote: The length of the message that can be sent in a single time is 256 characters\n");
    }
    
    lora_init_status = RET_TRUE;

    return ret;
}

static uint32 SpiLoraClose(void *dev)
{
    NULL_PARAM_CHECK(dev);

    return EOK;
}

static const struct LoraDevDone lora_done =
{
    .open = SpiLoraOpen,
    .close = SpiLoraClose,
    .write = SpiLoraWrite,
    .read = SpiLoraRead,
};

/**
 * This function supports to init spi_lora_dev
 *
 * @param bus_name spi bus name
 * @param dev_name spi dev name
 * @param drv_name spi drv name
 * @param flash_name flash dev name
 */
SpiLoraDeviceType SpiLoraInit(char *bus_name, char *dev_name, char *drv_name, char *lora_name)
{
    NULL_PARAM_CHECK(dev_name);
    NULL_PARAM_CHECK(drv_name);
    NULL_PARAM_CHECK(lora_name);
    NULL_PARAM_CHECK(bus_name);

    x_err_t ret;
    static HardwareDevType haldev;

    haldev = SpiDeviceFind(dev_name, TYPE_SPI_DEV);
    if (NONE == haldev) {
        KPrintf("SpiLoraInit find spi haldev %s error! \n", dev_name);
        return NONE;
    }

    SpiLoraDeviceType spi_lora_dev = (SpiLoraDeviceType)malloc(sizeof(struct SpiLoraDevice));
    if (NONE == spi_lora_dev) {
        KPrintf("SpiLoraInit malloc spi_lora_dev failed\n");
        free(spi_lora_dev);
        return NONE;
    }

    memset(spi_lora_dev, 0, sizeof(struct SpiLoraDevice));

    spi_lora_dev->spi_dev = CONTAINER_OF(haldev, struct SpiHardwareDevice, haldev);

    spi_lora_dev->lora_dev.spi_dev_flag = RET_TRUE;
    spi_lora_dev->lora_dev.haldev.dev_done = (struct HalDevDone *)&lora_done;

    spi_lora_dev->spi_dev->haldev.owner_bus->owner_driver = SpiDriverFind(drv_name, TYPE_SPI_DRV);
    if (NONE == spi_lora_dev->spi_dev->haldev.owner_bus->owner_driver) {
        KPrintf("SpiLoraInit find spi driver %s error! \n", drv_name);
        free(spi_lora_dev);
        return NONE;
    }

    ret = SpiDeviceRegister(&spi_lora_dev->lora_dev, spi_lora_dev->spi_dev->haldev.private_data, lora_name);
    if (EOK != ret) {
        KPrintf("SpiLoraInit SpiDeviceRegister device %s error %d\n", lora_name, ret);
        free(spi_lora_dev);   
        return NONE;
    }

    ret = SpiDeviceAttachToBus(lora_name, bus_name);
    if (EOK != ret) {
        KPrintf("SpiLoraInit SpiDeviceAttachToBus device %s error %d\n", lora_name, ret);
        free(spi_lora_dev);
        return NONE;
    }

    g_spi_lora_dev = &spi_lora_dev->spi_dev->haldev;

    return spi_lora_dev;
}

/**
 * This function supports to release spi_lora_dev
 *
 * @param spi_lora_dev spi lora descriptor
 */
uint32 SpiLoraRelease(SpiLoraDeviceType spi_lora_dev)
{
    NULL_PARAM_CHECK(spi_lora_dev);

    x_err_t ret;

    DeviceDeleteFromBus(spi_lora_dev->lora_dev.haldev.owner_bus, &spi_lora_dev->lora_dev.haldev);

    free(spi_lora_dev);

    return EOK;
}

int LoraSx12xxSpiDeviceInit(void)
{
#ifdef BSP_USING_SPI1

    if (NONE == SpiLoraInit(SPI_BUS_NAME_1, SPI_1_DEVICE_NAME_0, SPI_1_DRV_NAME, SX12XX_DEVICE_NAME)) {
        return ERROR;
    }

#endif

    return EOK;
}

//#define LORA_TEST
#ifdef LORA_TEST
/*Just for lora test*/
static struct Bus *bus;
static struct HardwareDev *dev;
static struct Driver *drv;

void LoraOpen(void)
{
    x_err_t ret = EOK;

    ret = LoraSx12xxSpiDeviceInit();
    if (EOK != ret) {
        KPrintf("LoraSx12xxSpiDeviceInit failed\n");
        return;
    }

    bus = BusFind(SPI_BUS_NAME_1);
    dev = BusFindDevice(bus, SX12XX_DEVICE_NAME);
    drv = BusFindDriver(bus, SPI_1_DRV_NAME);

    bus->match(drv, dev);
 
    ret = SpiLoraOpen(dev);
    if (EOK != ret) {
        KPrintf("LoRa init failed\n");
        return;
    }

    KPrintf("LoRa init succeed\n");

    return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                                                LoraOpen, LoraOpen,  open lora device and read parameters );

static void LoraReceive(void)
{
    struct BusBlockReadParam read_param;
    memset(&read_param, 0, sizeof(struct BusBlockReadParam));

    read_param.buffer = malloc(SPI_LORA_BUFFER_SIZE);

    SpiLoraRead(dev, &read_param);

    free(read_param.buffer);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0),
LoraReceive, LoraReceive,  lora wait message );

static void LoraSend(int argc, char *argv[])
{
    char Msg[SPI_LORA_BUFFER_SIZE] = {0};
    struct BusBlockWriteParam write_param;
    memset(&write_param, 0, sizeof(struct BusBlockWriteParam));

    if (argc == 2) {
        strncpy(Msg, argv[1], SPI_LORA_BUFFER_SIZE);
        write_param.buffer = Msg;
        write_param.size = strlen(Msg);

        SpiLoraWrite(dev, &write_param);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
LoraSend, LoraSend,  lora send message );
#endif
