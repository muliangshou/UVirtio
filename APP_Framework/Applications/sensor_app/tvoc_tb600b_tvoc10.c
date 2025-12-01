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
 * @file tvoc_tb600b_tvoc10.c
 * @brief tvoc10 example
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.15
 */

#ifdef ADD_XIZI_FETURES
# include <user_api.h>
#endif
#include <sensor.h>


/**
 * @description: Read a tvoc
 * @return 0
 */
void TvocTb600bTvoc10(void)
{
    struct SensorQuantity *tvoc = SensorQuantityFind(SENSOR_QUANTITY_TB600B_TVOC, SENSOR_QUANTITY_TVOC);
    SensorQuantityOpen(tvoc);
    int32_t result = 0;

    result = SensorQuantityRead(tvoc);

    printf("tvoc concentration is : %dppb\n", result);
    SensorQuantityClose(tvoc);
}