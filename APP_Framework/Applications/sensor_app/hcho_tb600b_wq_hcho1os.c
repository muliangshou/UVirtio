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
 * @file hcho_tb600b_wq_hcho1os.c
 * @brief hcho example
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.15
 */

#ifdef ADD_XIZI_FETURES
# include <user_api.h>
#endif
#include <sensor.h>


/**
 * @description: Read a hcho
 * @return 0
 */
void HchoTb600bHcho1os(void)
{
    struct SensorQuantity *hcho = SensorQuantityFind(SENSOR_QUANTITY_TB600B_HCHO, SENSOR_QUANTITY_HCHO);
    SensorQuantityOpen(hcho);
    int32_t result = 0;

    result = SensorQuantityRead(hcho);

    printf("tvoc concentration is : %dppb\n", result);
    SensorQuantityClose(hcho);
}