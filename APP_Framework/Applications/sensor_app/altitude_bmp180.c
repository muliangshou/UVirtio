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
 * @file altitude_bmp180.c
 * @brief BMP180 altitude example
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.12.23
 */

#include <user_api.h>
#include <sensor.h>

/**
 * @description: Read a altitude
 * @return 0
 */
void AltitudeBmp180(void)
{
    int32 altitude;
    struct SensorQuantity *p_altitude = SensorQuantityFind(SENSOR_QUANTITY_BMP180_ALTITUDE, SENSOR_QUANTITY_ALTITUDE);
    SensorQuantityOpen(p_altitude);
    altitude = SensorQuantityRead(p_altitude);

    printf("Altitude Pressure : %d Pa\n", altitude);

    PrivTaskDelay(1000);
    SensorQuantityClose(p_altitude);
}