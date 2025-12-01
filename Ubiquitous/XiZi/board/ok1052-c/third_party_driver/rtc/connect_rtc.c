/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
* @file connect_rtc.c
* @brief ok1052-c board rtc function and structure
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-01
*/

/*************************************************
File name: connect_rtc.c
Description: support ok1052-c board rtc configure and spi bus register function

History:
1. Date: 2022-03-01
Author: AIIT XUOS Lab
Modification:
1. change command for XUOS
2. add module codes for XUOS
*************************************************/

#include "board.h"
#include "bus_rtc.h"
#include "pin_mux.h"
#include "dev_rtc.h"
#include "connect_rtc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define rtc_print KPrintf

#define MAX_TIME_STR_SIZE 50

///////////RX8010///////////

#define RX8010_SEC     0x10
#define RX8010_MIN     0x11
#define RX8010_HOUR    0x12
#define RX8010_WDAY    0x13
#define RX8010_MDAY    0x14
#define RX8010_MONTH   0x15
#define RX8010_YEAR    0x16
#define RX8010_YEAR    0x16
#define RX8010_RESV17  0x17
#define RX8010_ALMIN   0x18
#define RX8010_ALHOUR  0x19
#define RX8010_ALWDAY  0x1A
#define RX8010_TCOUNT0 0x1B
#define RX8010_TCOUNT1 0x1C
#define RX8010_EXT     0x1D
#define RX8010_FLAG    0x1E
#define RX8010_CTRL    0x1F

/* 0x20 to 0x2F are user registers */
#define RX8010_RESV30  0x30
#define RX8010_RESV31  0x31
#define RX8010_IRQ     0x32

#define RX8010_EXT_WADA  0x04 //BIT(3)

#define RX8010_FLAG_VLF  0x02 //BIT(1)
#define RX8010_FLAG_AF   0x04 //BIT(3)
#define RX8010_FLAG_TF   0x08 //BIT(4)
#define RX8010_FLAG_UF   0x10 //BIT(5)

#define RX8010_CTRL_AIE  0x04 //BIT(3)
#define RX8010_CTRL_UIE  0x10 //BIT(5)
#define RX8010_CTRL_STOP 0x20 //BIT(6)
#define RX8010_CTRL_TEST 0x40 //BIT(7)

#define RX8010_ALARM_AE  0x40 //BIT(7)

#define RX8010_TEST_TIME 10000

#define BCD_DATA_LEN     20

// change BIN format to BCD format
#define TO_BCD(_n) (((_n / 10) << 4) | (_n % 10))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * Code
 ******************************************************************************/

// change BCD format date to BIN format
uint8_t bcd2bin(uint8_t data)
{
    int i = 0;
    uint8_t ret = 0;
    uint8_t mask[4] = {0x01, 0x02, 0x04, 0x08};

    //LOW
    for(i = 0; i < 4; i++)
    {
        if(mask[i] & data)
        {
            ret += mask[i];
        }
    }

    //HIGH
    for(i = 0; i < 4; i++)
    {
        if(mask[i] & (data >> 4))
        {
            ret += (mask[i] * 10);
        }
    }

    return ret;
}

// 8010 initialization
int RtcInit(void)
{
    uint8_t flag = 0;
    uint8_t data = 0;
    uint8_t ctrl[2];
    int need_clear = 0, err = 0;
    err = RtcI2cRead(I2C_RTC_BASE, RX8010_FLAG, &flag, 1);
    flag &= ~(RX8010_FLAG_VLF);
    err = RtcI2cWrite(I2C_RTC_BASE, RX8010_FLAG, &flag, 1);
    /* Initialize reserved registers as specified in datasheet */
    data = 0xD8;
    err = RtcI2cWrite(I2C_RTC_BASE, RX8010_RESV17, &data, 1);
    data = 0x00;
    err = RtcI2cWrite(I2C_RTC_BASE, RX8010_RESV30, &data, 1);
    data = 0x08;
    err = RtcI2cWrite(I2C_RTC_BASE, RX8010_RESV31, &data, 1);
    data = 0x00;
    err = RtcI2cWrite(I2C_RTC_BASE, RX8010_IRQ, &data, 1);
    err = RtcI2cRead(I2C_RTC_BASE, RX8010_FLAG, ctrl, 2);

    if(ctrl[0] & RX8010_FLAG_VLF)
    {
        rtc_print("\r\n Frequency stop was detected\r\n");
    }

    if(ctrl[0] & RX8010_FLAG_AF)
    {
        rtc_print("\r\n Alarm was detected\r\n");
        need_clear = 1;
    }

    if(ctrl[0] & RX8010_FLAG_TF)
    {
        need_clear = 1;
    }

    if(ctrl[0] & RX8010_FLAG_UF)
    {
        need_clear = 1;
    }

    if(need_clear)
    {
        ctrl[0] &= ~(RX8010_FLAG_AF | RX8010_FLAG_TF | RX8010_FLAG_UF);
        err = RtcI2cWrite(I2C_RTC_BASE, RX8010_FLAG, ctrl,1);

        if(!err)
        {
            return err;
        }
    }

    return err;
}

// check format and get BCD format date like 2018-06-21 16:29:30
int RtcGetBcdDate(uint8_t* date, uint8_t* bcd_date)
{
    int i;
    int temp_date[6];

    if(sscanf(date, "20%2d-%2d-%2d %2d:%2d:%2d",
              &temp_date[5],
              &temp_date[4],
              &temp_date[3],
              &temp_date[2],
              &temp_date[1],
              &temp_date[0]) == EOF)
    {
        rtc_print("i2c %s failed\n", __func__);
        return -1;
    }

    for(i = 0; i < 6; i++)
    {
        bcd_date[i] = TO_BCD(temp_date[i]);
    }

    return 0;
}

// setup time
int RtcSetTime(uint8_t* asc_date)
{
    uint8_t bcd_date[6];
    int ret, err;

    if(RtcGetBcdDate(asc_date, bcd_date))
    {
        rtc_print("\r\n Date format error! \r\n");
        return -1;
    }

    err = RtcI2cWrite(I2C_RTC_BASE, RX8010_SEC, bcd_date, 3);
    err |= RtcI2cWrite(I2C_RTC_BASE, RX8010_MDAY, &bcd_date[3], 3);
    return err;
}

// get rx8010 time
int RtcGetTime(struct tm *ct)
{
    uint8_t rtc_data[7];
    uint8_t time_str[7];
    uint8_t flag_reg;
    int err;
    err = RtcI2cRead(I2C_RTC_BASE, RX8010_FLAG, &flag_reg, 1);

    if(flag_reg & RX8010_FLAG_VLF)
    {
        rtc_print("\r\n Frequency stop was detected\r\n");
        return 1;
    }

    err = RtcI2cRead(I2C_RTC_BASE, RX8010_SEC, rtc_data, 7);
    time_str[0] = bcd2bin(rtc_data[RX8010_SEC - RX8010_SEC] & 0x7f);
    time_str[1] = bcd2bin(rtc_data[RX8010_MIN - RX8010_SEC] & 0x7f);
    time_str[2] = bcd2bin(rtc_data[RX8010_HOUR - RX8010_SEC] & 0x3f);
    time_str[4] = bcd2bin(rtc_data[RX8010_MDAY - RX8010_SEC] & 0x3f);
    time_str[5] = bcd2bin(rtc_data[RX8010_MONTH - RX8010_SEC] & 0x1f);
    time_str[6] = bcd2bin(rtc_data[RX8010_YEAR - RX8010_SEC]);
    time_str[3] = rtc_data[RX8010_WDAY - RX8010_SEC] & 0x7f;

    rtc_print("RX8010 Time: 20%d%d-%d%d-%d%d %d%d:%d%d:%d%d\r\n",
              time_str[6]/10, time_str[6]%10, time_str[5]/10, time_str[5]%10, time_str[4]/10, time_str[4]%10,
              time_str[2]/10, time_str[2]%10, time_str[1]/10, time_str[1]%10, time_str[0]/10, time_str[0]%10);

    ct->tm_year = time_str[6];
    ct->tm_mon = time_str[5];
    ct->tm_mday = time_str[4];
    ct->tm_wday = time_str[3];
    ct->tm_hour = time_str[2];
    ct->tm_min = time_str[1];
    ct->tm_sec = time_str[0];
    return 0;
}

static int GetWeekDay(int year, int month, int day)
{
    if(month==1||month==2)
    {
        year -=1;
        month +=12;
    }

    return (day+1+2*month+3*(month+1)/5+year+(year/4)-year/100+year/400)%7+1;
}

static uint32 RtcConfigure(void* drv, struct BusConfigureInfo* configure_info)
{
    NULL_PARAM_CHECK(drv);
    struct RtcDriver* rtc_drv = (struct RtcDriver*)drv;
    struct RtcDrvConfigureParam* drv_param = (struct RtcDrvConfigureParam*)configure_info->private_data;
    int cmd = drv_param->rtc_operation_cmd;
    time_t* time = drv_param->time;

    switch(cmd)
    {
        case OPER_RTC_GET_TIME:
        {
            struct tm ct;
            RtcGetTime(&ct);
            *time = mktime(&ct);
        }
        break;

        case OPER_RTC_SET_TIME:
        {
            char time_str[MAX_TIME_STR_SIZE] = {0};
            struct tm *ct = localtime(time);
            strftime(time_str, MAX_TIME_STR_SIZE, "%y-%m-%d %H:%M:%S", ct);
            RtcSetTime(time_str);
        }
        break;
    }

    return EOK;
}

int RtcConfiguration(void)
{
    BOARD_InitI2C1Pins();
    RtcI2cInit();
    RtcInit();
    return 0;
}

/*manage the rtc device operations*/
static const struct RtcDevDone dev_done =
{
    .open = NONE,
    .close = NONE,
    .write = NONE,
    .read = NONE,
};

static int BoardRtcBusInit(struct RtcBus* rtc_bus, struct RtcDriver* rtc_driver)
{
    x_err_t ret = EOK;
    /*Init the rtc bus */
    ret = RtcBusInit(rtc_bus, RTC_BUS_NAME);

    if(EOK != ret)
    {
        KPrintf("hw_rtc_init RtcBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the rtc driver*/
    ret = RtcDriverInit(rtc_driver, RTC_DRV_NAME);

    if(EOK != ret)
    {
        KPrintf("hw_rtc_init RtcDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the rtc driver to the rtc bus*/
    ret = RtcDriverAttachToBus(RTC_DRV_NAME, RTC_BUS_NAME);

    if(EOK != ret)
    {
        KPrintf("hw_rtc_init RtcDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

/*Attach the rtc device to the rtc bus*/
static int BoardRtcDevBend(void)
{
    x_err_t ret = EOK;
    static struct RtcHardwareDevice rtc_device;
    memset(&rtc_device, 0, sizeof(struct RtcHardwareDevice));
    rtc_device.dev_done = &(dev_done);
    ret = RtcDeviceRegister(&rtc_device, NONE, RTC_DEVICE_NAME);

    if(EOK != ret)
    {
        KPrintf("hw_rtc_init RtcDeviceInit device %s error %d\n", RTC_DEVICE_NAME, ret);
        return ERROR;
    }

    ret = RtcDeviceAttachToBus(RTC_DEVICE_NAME, RTC_BUS_NAME);

    if(EOK != ret)
    {
        KPrintf("hw_rtc_init RtcDeviceAttachToBus device %s error %d\n", RTC_DEVICE_NAME, ret);
        return ERROR;
    }

    return  ret;
}

int Imxrt1052HwRtcInit(void)
{
    x_err_t ret = EOK;
    static struct RtcBus rtc_bus;
    memset(&rtc_bus, 0, sizeof(struct RtcBus));
    static struct RtcDriver rtc_driver;
    memset(&rtc_driver, 0, sizeof(struct RtcDriver));
    rtc_driver.configure = &(RtcConfigure);
    ret = BoardRtcBusInit(&rtc_bus, &rtc_driver);

    if(EOK != ret)
    {
        KPrintf("hw_rtc_init error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardRtcDevBend();

    if(EOK != ret)
    {
        KPrintf("hw_rtc_init error ret %u\n", ret);
        return ERROR;
    }

    RtcConfiguration();
    return ret;
}

void RtcTestRx8010(int argc, char* argv[])
{
    if(argc == 2)
    {
        if(RtcSetTime(argv[1]) == 0)
        {
            RtcGetTime(NULL);
        }
    }
    else
    {
        RtcGetTime(NULL);
    }
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)| SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)| SHELL_CMD_PARAM_NUM(3),
                 rtc, RtcTestRx8010, i2c rtc "date time");

