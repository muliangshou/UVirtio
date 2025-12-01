/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
* @file connect_sdio.c
* @brief support sdio function using bus driver framework on xidatong board
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2022-01-24
*/

/*************************************************
File name: connect_sdio.c
Description: support imxrt1052-board sd card configure and sdio bus register function
Others: take SDK_2.6.1_MIMXRT1052xxxxB/boards/evkbimxrt1050/driver_examples/sdcard/polling/sdcard_polling.c for references
History: 
1. Date: 2022-01-24
Author: AIIT XUOS Lab
Modification: 
1. support imxrt1052-board sdio configure, write and read
2. support imxrt1052-board sdio bus device and driver register
*************************************************/

#include <connect_sdio.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SDCARD_POWER_CTRL_FUNCTION_EXIST

/*! @brief Data block count accessed in card */
#define DATA_BLOCK_COUNT (5U)
/*! @brief Start data block number accessed in card */
#define DATA_BLOCK_START (2U)
/*! @brief Data buffer size. */
#define DATA_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)

#define BOARD_USDHC_SDCARD_POWER_CONTROL(state) \
    (GPIO_PinWrite(BOARD_SD_POWER_RESET_GPIO, BOARD_SD_POWER_RESET_GPIO_PIN, state))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void BoardPowerOffSdCard(void);
static void BoardPowerOnSdCard(void);

/*!
 * @brief printf the card information log.
 *
 * @param card Card descriptor.
 */
static void CardInformationLog(sd_card_t *card);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* @brief decription about the read/write buffer
 * The size of the read/write buffer should be a multiple of 512, since SDHC/SDXC card uses 512-byte fixed
 * block length and this driver example is enabled with a SDHC/SDXC card.If you are using a SDSC card, you
 * can define the block length by yourself if the card supports partial access.
 * The address of the read/write buffer should align to the specific DMA data buffer address align value if
 * DMA transfer is used, otherwise the buffer address is not important.
 * At the same time buffer address/size should be aligned to the cache line size if cache is supported.
 */
/*! @brief Data written to the card */
SDK_ALIGN(uint8_t g_data_write[SDK_SIZEALIGN(DATA_BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));
/*! @brief Data read from the card */
SDK_ALIGN(uint8_t g_data_read[SDK_SIZEALIGN(DATA_BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));

/*! @brief SDMMC host detect card configuration */
static const sdmmchost_detect_card_t s_sdcard_detect = {
#ifndef BOARD_SD_DETECT_TYPE
    .cdType = kSDMMCHOST_DetectCardByGpioCD,
#else
    .cdType = BOARD_SD_DETECT_TYPE,
#endif
    .cdTimeOut_ms = (~0U),
};

/*! @brief SDMMC card power control configuration */
#if defined SDCARD_POWER_CTRL_FUNCTION_EXIST
static const sdmmchost_pwr_card_t s_sdcard_pwr_ctrl = {
    .powerOn          = BoardPowerOnSdCard,
    .powerOnDelay_ms  = 0U,
    .powerOff         = BoardPowerOffSdCard,
    .powerOffDelay_ms = 0U,
};
#endif

/*! @brief SDMMC card power control configuration */
#if defined SDCARD_SWITCH_VOLTAGE_FUNCTION_EXIST
static const sdmmchost_card_switch_voltage_func_t s_sdcard_voltage_switch = {
    .cardSignalLine1V8 = BOARD_USDHC_Switch_VoltageTo1V8,
    .cardSignalLine3V3 = BOARD_USDHC_Switch_VoltageTo3V3,
};
#endif

/*! @brief Card descriptor. */
static sd_card_t g_sd;
static int sd_lock = -1;

static void BoardUSDHCClockConfiguration(void)
{
    CLOCK_InitSysPll(&sysPllConfig_BOARD_BootClockRUN);
    /*configure system pll PFD0 fractional divider to 24, output clock is 528MHZ * 18 / 24 = 396 MHZ*/
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 24U);
    /* Configure USDHC clock source and divider */
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
}

static void BoardPowerOffSdCard(void)
{
    /*
        Do nothing here.

        SD card will not be detected correctly if the card VDD is power off,
       the reason is caused by card VDD supply to the card detect circuit, this issue is exist on EVK board rev A1 and
       A2.

        If power off function is not implemented after soft reset and prior to SD Host initialization without
       remove/insert card,
       a UHS-I card may not reach its highest speed mode during the second card initialization.
       Application can avoid this issue by toggling the SD_VDD (GPIO) before the SD host initialization.
    */
}

static void BoardPowerOnSdCard(void)
{
    BOARD_USDHC_SDCARD_POWER_CONTROL(1);
}

static void CardInformationLog(sd_card_t *card)
{
    NULL_PARAM_CHECK(card);

    KPrintf("\r\nCard size %d * %d bytes\r\n", card->blockCount, card->blockSize);
    KPrintf("\r\nWorking condition:\r\n");
    if (card->operationVoltage == kCARD_OperationVoltage330V) {
        KPrintf("\r\n  Voltage : 3.3V\r\n");
    } else if (card->operationVoltage == kCARD_OperationVoltage180V) {
        KPrintf("\r\n  Voltage : 1.8V\r\n");
    }

    if (card->currentTiming == kSD_TimingSDR12DefaultMode) {
        if (card->operationVoltage == kCARD_OperationVoltage330V) {
            KPrintf("\r\n  Timing mode: Default mode\r\n");
        } else if (card->operationVoltage == kCARD_OperationVoltage180V) {
            KPrintf("\r\n  Timing mode: SDR12 mode\r\n");
        }
    } else if (card->currentTiming == kSD_TimingSDR25HighSpeedMode) {
        if (card->operationVoltage == kCARD_OperationVoltage180V) {
            KPrintf("\r\n  Timing mode: SDR25\r\n");
        } else {
            KPrintf("\r\n  Timing mode: High Speed\r\n");
        }
    } else if (card->currentTiming == kSD_TimingSDR50Mode) {
        KPrintf("\r\n  Timing mode: SDR50\r\n");
    } else if (card->currentTiming == kSD_TimingSDR104Mode) {
        KPrintf("\r\n  Timing mode: SDR104\r\n");
    } else if (card->currentTiming == kSD_TimingDDR50Mode) {
        KPrintf("\r\n  Timing mode: DDR50\r\n");
    }

    KPrintf("\r\n  Freq : %d HZ\r\n", card->busClock_Hz);
}

static uint32 SdioConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    if (configure_info->configure_cmd == OPER_BLK_GETGEOME) {
        NULL_PARAM_CHECK(configure_info->private_data);
        struct DeviceBlockArrange *args = (struct DeviceBlockArrange *)configure_info->private_data;

        args->size_perbank = g_sd.blockSize;
        args->block_size = g_sd.blockSize;
        args->bank_num = g_sd.blockCount;
    }

    return EOK;
}

static uint32 SdioOpen(void *dev)
{
    NULL_PARAM_CHECK(dev);

    if(sd_lock >= 0) {
        KSemaphoreDelete(sd_lock);
    }
    sd_lock = KSemaphoreCreate(1);
    if (sd_lock < 0){
        return ERROR;
    }

    return EOK;
}

static uint32 SdioClose(void *dev)
{
    NULL_PARAM_CHECK(dev);

    KSemaphoreDelete(sd_lock);

    return EOK;
}

static uint32 SdioRead(void *dev, struct BusBlockReadParam *read_param)
{
    uint8 ret = EOK;
    uint32 sector = read_param->pos;
    uint32 block_num = read_param->size;
    uint8 *read_buffer = (uint8 *)read_param->buffer;

    KSemaphoreObtain(sd_lock, WAITING_FOREVER);

    if (kStatus_Success != SD_ReadBlocks(&g_sd, read_buffer, sector, block_num)) {
        KPrintf("Read multiple data blocks failed.\r\n");
        return 0;
    }

    KSemaphoreAbandon(sd_lock);

    return read_param->size;
}

static uint32 SdioWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    uint8 ret = EOK;
    uint32 sector = write_param->pos;
    uint32 block_num = write_param->size;
    const uint8 *write_buffer = (uint8 *)write_param->buffer;

    KSemaphoreObtain(sd_lock, WAITING_FOREVER);

    if (kStatus_Success != SD_WriteBlocks(&g_sd, write_buffer, sector, block_num)) {
        KPrintf("Write multiple data blocks failed.\r\n");
        return 0;
    }

    KSemaphoreAbandon(sd_lock);

    return write_param->size;
}

static int SdioControl(struct HardwareDev *dev, struct HalDevBlockParam *block_param)
{
    NULL_PARAM_CHECK(dev);

    if (OPER_BLK_GETGEOME == block_param->cmd) {
        block_param->dev_block.size_perbank = g_sd.blockSize;
        block_param->dev_block.block_size = g_sd.blockSize;
        block_param->dev_block.bank_num = g_sd.blockCount;
    }

    return EOK;
}

static struct SdioDevDone dev_done = 
{
    SdioOpen,
    SdioClose,
    SdioWrite,
    SdioRead,
};

#if defined(FS_VFS) && defined(MOUNT_SDCARD_FS)
#include <iot-vfs.h>

/**
 * @description: Mount SD card
 * @return 0
 */
static int MountSDCardFs(enum FilesystemType fs_type)
{
    if (MountFilesystem(SDIO_BUS_NAME, SDIO_DEVICE_NAME, SDIO_DRIVER_NAME, fs_type, "/") == 0)
        KPrintf("Sd card mount to '/'");
    else
        KPrintf("Sd card mount to '/' failed!");
    
    return 0;
}
#endif

static void SdCardAttach(void)
{
    bool is_read_only;
    static sd_card_t *card = &g_sd;
    
    KPrintf("\r\nCard inserted.\r\n");
    /* reset host once card re-plug in */
    SD_HostReset(&(card->host));
    /* power on the card */
    SD_PowerOnCard(card->host.base, card->usrParam.pwr);
    KPrintf("Power on done\n");

    /* Init card. */
    if (SD_CardInit(card)) {
        KPrintf("\r\nSD card init failed.\r\n");
        return;
    }

    /* card information log */
    CardInformationLog(card);

    /* Check if card is readonly. */
    is_read_only = SD_CheckReadOnly(card);

#ifdef MOUNT_SDCARD_FS
    /*mount file system*/
    MountSDCardFs(MOUNT_SDCARD_FS_TYPE);
#endif
}

static void SdCardDetach(void)
{
    /*unmount file system*/
    KPrintf("\r\nCard detect extracted.\r\n");

#ifdef MOUNT_SDCARD_FS
    UnmountFileSystem("/");
#endif
}

static uint8 SdCardReadCd(void)
{
	BusType pin;

    pin = BusFind(PIN_BUS_NAME);

    pin->owner_haldev = BusFindDevice(pin, PIN_DEVICE_NAME);

    struct PinStat PinStat;

    struct BusBlockReadParam read_param;
    read_param.buffer = (void *)&PinStat;

    PinStat.pin = IMXRT_GET_PIN(2, 28);
    
    return BusDevReadData(pin->owner_haldev, &read_param);
}

static void SdCardTask(void* parameter)
{
    static int sd_card_status = 0;

    while (1) {
        if (!SdCardReadCd()) {
            if (!sd_card_status) {
                SdCardAttach();
                sd_card_status = 1;
            }
        } else {
            if (sd_card_status) {
                SdCardDetach();
                sd_card_status = 0;
            }
        }
    }
}

#ifdef MOUNT_SDCARD
int MountSDCard()
{
    int sd_card_task = 0;
    sd_card_task = KTaskCreate("sd_card", SdCardTask, NONE,
                           SD_CARD_STACK_SIZE, 8);
    if(sd_card_task < 0) {		
		KPrintf("sd_card_task create failed ...%s %d.\n", __FUNCTION__,__LINE__);
		return ERROR;
	}

    StartupKTask(sd_card_task);

    return EOK;
}
#endif

int Imxrt1052HwSdioInit(void)
{
    x_err_t ret = EOK;
    
    static struct SdioBus sdio_bus;
    static struct SdioDriver sdio_drv;
    static struct SdioHardwareDevice sdio_dev;
    static sd_card_t *card = &g_sd;

    memset(&sdio_bus, 0, sizeof(struct SdioBus));
    memset(&sdio_drv, 0, sizeof(struct SdioDriver));
    memset(&sdio_dev, 0, sizeof(struct SdioHardwareDevice));

    BoardUSDHCClockConfiguration();

    card->host.base           = SD_HOST_BASEADDR;
    card->host.sourceClock_Hz = SD_HOST_CLK_FREQ;

    /* card detect type */
    card->usrParam.cd = &s_sdcard_detect;

#if defined SDCARD_POWER_CTRL_FUNCTION_EXIST
    card->usrParam.pwr = &s_sdcard_pwr_ctrl;
#endif

#if defined DEMO_SDCARD_SWITCH_VOLTAGE_FUNCTION_EXIST
    card->usrParam.cardVoltage = &s_sdcard_voltage_switch;
#endif

    /* SD host init function */
    if (SD_HostInit(card) != kStatus_Success) {
        KPrintf("\r\nSD host init fail\r\n");
        return ERROR;
    }
    
    ret = SdioBusInit(&sdio_bus, SDIO_BUS_NAME);
    if (ret != EOK) {
        KPrintf("Sdio bus init error %d\n", ret);
        return ERROR;
    }

    ret = SdioDriverInit(&sdio_drv, SDIO_DRIVER_NAME);
    if (ret != EOK) {
        KPrintf("Sdio driver init error %d\n", ret);
        return ERROR;
    }
    ret = SdioDriverAttachToBus(SDIO_DRIVER_NAME, SDIO_BUS_NAME);
    if (ret != EOK) {
        KPrintf("Sdio driver attach error %d\n", ret);
        return ERROR;
    }

    sdio_dev.dev_done = &dev_done;
    sdio_dev.haldev.dev_block_control = SdioControl;
    ret = SdioDeviceRegister(&sdio_dev, SDIO_DEVICE_NAME);
    if (ret != EOK) {
        KPrintf("Sdio device register error %d\n", ret);
        return ERROR;
    }
    ret = SdioDeviceAttachToBus(SDIO_DEVICE_NAME, SDIO_BUS_NAME);
    if (ret != EOK) {
        KPrintf("Sdio device register error %d\n", ret);
        return ERROR;
    }

    return ret;
}
