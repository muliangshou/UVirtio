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
 * @file pm2_5_ps5308.c
 * @brief PS5308 PM2.5 example
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.28
 */

#ifdef ADD_XIZI_FETURES
# include <user_api.h>
#endif

#include <sensor.h>

/**
 * @description: Read a PM2.5
 * @return 0
 */
void Pm25Ps5308(void)
{
    struct SensorQuantity *pm2_5 = SensorQuantityFind(SENSOR_QUANTITY_PS5308_PM2_5, SENSOR_QUANTITY_PM);
    SensorQuantityOpen(pm2_5);
    PrivTaskDelay(2000);
    printf("PM2.5 : %d ug/m³\n", SensorQuantityRead(pm2_5));
    SensorQuantityClose(pm2_5);
}