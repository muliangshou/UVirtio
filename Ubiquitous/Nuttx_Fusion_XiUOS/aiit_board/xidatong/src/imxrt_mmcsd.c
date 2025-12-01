/*
* Copyright (c) 2020 AIIT XUOS Lab
* XiOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
 * @file imxrt_sdhc_automount.c
 * @brief imxrt board sd card automount
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.04.12
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <debug.h>

#include "xidatong.h"

#ifdef CONFIG_IMXRT_USDHC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_sdmmc_initialize
 *
 * Description:
 *   Initialize the SDHC SD card slot
 *
 ****************************************************************************/

int nsh_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdmmc;
  int ret = 0;

  /* Get an instance of the SDIO interface */

  sdmmc = imxrt_usdhc_initialize(0);
  if (!sdmmc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
      return -ENODEV;
    }
  /* Bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(0, sdmmc);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_XIDATONG_SDIO_AUTOMOUNT
      imxrt_automount_initialize();
      imxrt_usdhc_set_sdio_card_isr(sdmmc, imxrt_sdhc_automount_event, NULL);
#else
      imxrt_usdhc_set_sdio_card_isr(sdmmc, NULL, NULL);
#endif

  return OK;
}
#endif