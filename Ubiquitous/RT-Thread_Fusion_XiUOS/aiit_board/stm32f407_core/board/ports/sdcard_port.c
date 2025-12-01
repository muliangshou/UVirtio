/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-13     balanceTWK   add sdcard port file
 */

#include <rtthread.h>
#include <rtdevice.h>
#ifdef BSP_USING_SDCARD
#include <board.h>
#include <spi_msd.h>
#include <dfs_elm.h>
#include <dfs_fs.h>
#include <drv_spi.h>
#include <dfs_posix.h>

#define DBG_TAG "sdcard"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

int sd_mount()
{
    GPIO_TypeDef *cs_gpiox;
    uint16_t cs_pin;
    int ret = 0;
    cs_gpiox = (GPIO_TypeDef *)((rt_base_t)GPIOA + (rt_base_t)(SDCARD_SPI2_CS_PIN / 16) * 0x0400UL);
    cs_pin = (uint16_t)(1 << SDCARD_SPI2_CS_PIN % 16);
    rt_hw_spi_device_attach(SDCARD_SPI_BUS_NAME, "spi20", cs_gpiox, cs_pin);
    ret = msd_init("sd", "spi20");
    if(RT_EOK == ret)
    {
        #ifdef RT_USING_DFS_ROMFS
        if(dfs_mount("sd", "/SD", "elm", 0, 0) == 0)
        #else
        if(dfs_mount("sd", "/", "elm", 0, 0) == 0)
        #endif
        {
            LOG_I("Mount sd card  successfully"); 
            return RT_EOK;
        }
        else
        {
            LOG_E("Mount fail !!!");
            return -RT_ERROR;
        }
    }
    LOG_E("msd_init fail !!!");
    return -RT_ERROR;    
}
INIT_APP_EXPORT(sd_mount);
#endif /* BSP_USING_SDCARD */

