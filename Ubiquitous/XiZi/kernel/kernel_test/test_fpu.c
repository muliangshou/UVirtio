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
* @file test_fpu.c
* @brief support to test touch function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2022-01-11
*/
#include <xizi.h>
void FpuTest(void)
{
    float i = 8.25;
    KPrintf("float i = 8.25 ['i %s 8' && 'i %s 9']\n",i > 8 ? ">" : "<",i < 9 ? "<" : ">");
    float add = i + 5.1;
    KPrintf("float add(13.35) = i + 5.1 ['add %s 13' && 'add %s 14']\n",add > 13 ? ">" : "<",add < 14 ? "<" : ">");
    float sub = i - 5.1;
    KPrintf("float sub(3.15) = i - 5.1 ['sub %s 3' && 'sub %s 4']\n",sub > 3 ? ">" : "<",sub < 4 ? "<" : ">");
    float mul = i * 5.1;
    KPrintf("float mul(42.075) = i * 5.1 ['mul %s 42' && 'mul %s 43']\n",mul > 42 ? ">" : "<",mul < 43 ? "<" : ">");
    float div = i / 5.1;
    KPrintf("float div(1.617) = i / 5.1 ['div %s 1' && 'div %s 2']\n",div > 1 ? ">" : "<",div < 2 ? "<" : ">");
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0),FpuTest, FpuTest,  Close AC task );