
#include <rtdevice.h>
#include <rtthread.h>
#include "drv_gpio.h"
#include <board.h>
#include <dfs_elm.h>
#include <dfs_posix.h>
#define DBG_TAG "sdcard"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
int sd_mount()
{
    rt_uint32_t result;
    result = mmcsd_wait_cd_changed(RT_TICK_PER_SECOND);
    if (result == MMCSD_HOST_PLUGED)
    {
        /* mount sd card fat partition 1 as root directory */
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
        {
            LOG_I("File System initialized!\n");
            return RT_EOK;
        }
        else
        {
            LOG_E("File System init failed!\n");
            return -RT_ERROR;
        }
            
    }
    LOG_E("msd_init fail !!!");
    return -RT_ERROR;    
}
INIT_APP_EXPORT(sd_mount);