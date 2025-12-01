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
 * @file windspeed_qs_fs.c
 * @brief qs-fx wind direction example
 * @version 1.1
 * @author AIIT XUOS Lab
 * @date 2021.12.14
 */

#include <transform.h>
#include <sensor.h>

/**
 * @description: Read a wind speed
 * @return 0
 */
void WindSpeedQsFs(void)
{
    struct SensorQuantity *wind_speed = SensorQuantityFind(SENSOR_QUANTITY_QS_FS_WINDSPEED, SENSOR_QUANTITY_WINDSPEED);
    SensorQuantityOpen(wind_speed);
    PrivTaskDelay(2000);
    uint16 result = SensorQuantityRead(wind_speed);
    printf("wind speed : %d.%d m/s\n", result/10, result%10);
    SensorQuantityClose(wind_speed);
}
#ifdef ADD_XIZI_FETURES
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, WindSpeedQsFs, WindSpeedQsFs, WindSpeedQsFs function);
#endif

