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
 * @file temperature_hs300x.c
 * @brief HS300x temperature example
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.04.23
 */

#ifdef ADD_XIZI_FETURES
# include <user_api.h>
#endif

#include <sensor.h>

/**
 * @description: Read a temperature
 * @return 0
 */
void TempHs300x(void)
{
    int i = 0;
    int32 temperature;
    struct SensorQuantity *temp = SensorQuantityFind(SENSOR_QUANTITY_HS300X_TEMPERATURE, SENSOR_QUANTITY_TEMP);
    SensorQuantityOpen(temp);
    for (i = 0; i < 100; i ++) {
        temperature = SensorQuantityRead(temp);
        if (temperature > 0)
            printf("Temperature : %d.%d ℃\n", temperature/10, temperature%10);
        else
            printf("Temperature : %d.%d ℃\n", temperature/10, -temperature%10);

        PrivTaskDelay(5000);
    }

    SensorQuantityClose(temp);
}
