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
 * @file ch4_as830.c
 * @brief CH4 AS830 example
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.10
 */

#ifdef ADD_XIZI_FETURES
# include <user_api.h>
#endif
#include <sensor.h>

/**
 * @description: Read a ch4
 * @return 0
 */
void Ch4As830(void)
{
    struct SensorQuantity *ch4 = SensorQuantityFind(SENSOR_QUANTITY_AS830_CH4, SENSOR_QUANTITY_CH4);
    SensorQuantityOpen(ch4);
    printf("CH4 : %d %%LTL\n", SensorQuantityRead(ch4));
    SensorQuantityClose(ch4);
}