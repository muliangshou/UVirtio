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
 * @file pm10_0_ps5308.c
 * @brief PS5308 PM10.0 example
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.04.23
 */

#ifdef ADD_XIZI_FETURES
# include <user_api.h>
#endif

#include <sensor.h>

/**
 * @description: Read a PM10
 * @return 0
 */
void Pm100Ps5308(void)
{
    struct SensorQuantity *pm10_0 = SensorQuantityFind(SENSOR_QUANTITY_PS5308_PM10, SENSOR_QUANTITY_PM);
    SensorQuantityOpen(pm10_0);
    PrivTaskDelay(2000);
    printf("PM10 : %d ug/m³\n", SensorQuantityRead(pm10_0));
    SensorQuantityClose(pm10_0);
}