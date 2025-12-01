#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef BSP_USING_SDRAM
#include <sdram_port.h>

#define DRV_DEBUG
#define LOG_TAG             "drv.sram"
#include <drv_log.h>

static void sdram_test2(void)
{
    char *p =NULL;
	p = rt_malloc(1024 * 1024 * 1);
	if(p == NULL)
	{
		LOG_E("apply for 1MB memory fail ~!!!");
	}
	else
	{
		LOG_D("appyle for 1MB memory success!!!");
	}
	rt_free(p);
}
MSH_CMD_EXPORT(sdram_test2, sdram test2);

#endif