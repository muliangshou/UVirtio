#include <rtthread.h>
#if defined(BSP_USING_SPI_FLASH)
#include "spi_flash_sfud.h"
#include "spi_flash.h"
#include "drv_spi.h"
#include <board.h>
#include <spi_msd.h>
#include <dfs_elm.h>
#include <dfs_fs.h>
#include <drv_spi.h>
#include <dfs_posix.h>
#include <rtdbg.h>

static int rt_hw_spi_flash_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    rt_hw_spi_device_attach("spi2", "spi30", GPIOB, GPIO_PIN_14);

    if (RT_NULL == rt_sfud_flash_probe("W25Q128", "spi30"))
    {
        LOG_E("W25Q128 flash spi30  fail !!!");
        return -RT_ERROR;
    };

    #ifdef RT_USING_DFS_ROMFS
    if (dfs_mount("W25Q128", "/FLASH", "elm", 0, 0) == RT_EOK)
    #else
    if (dfs_mount("W25Q128", "/", "elm", 0, 0) == RT_EOK)
    #endif
    {
        LOG_I("Mount  flash  successfully"); 
        return RT_EOK;
    }
    else
    {
        LOG_E("Mount fail !!!");
        return -RT_ERROR;
    }      
}
INIT_APP_EXPORT(rt_hw_spi_flash_init);
#endif

