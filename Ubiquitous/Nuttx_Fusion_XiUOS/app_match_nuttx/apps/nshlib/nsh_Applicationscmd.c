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
 * @file nsh_Applicationscmd.c
 * @brief nsh cmd function support
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.03.17
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#include "nsh.h"
#include "nsh_console.h"

extern int FrameworkInit(void);

/****************************************************************************
 * Name: cmd_Hcho1os
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_HCHO_TB600B_WQ_HCHO1OS) && !defined(CONFIG_NSH_DISABLE_HCHO_TB600B_WQ_HCHO1OS)
extern void HchoTb600bHcho1os(void);
int cmd_Hcho1os(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    HchoTb600bHcho1os();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_Tvoc10
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_TVOC_TB600B_TVOC10) && !defined(CONFIG_NSH_DISABLE_TB600B_TVOC10)
extern void TvocTb600bTvoc10(void);
int cmd_Tvoc10(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    TvocTb600bTvoc10();
    return OK;
}
#endif


/****************************************************************************
 * Name: cmd_Iaq10
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_IAQ) && !defined(CONFIG_NSH_DISABLE_TB600B_IAQ10)
extern void IaqTb600bIaq10(void);
int cmd_Iaq10(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    IaqTb600bIaq10();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_As830
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_CH4_AS830) && !defined(CONFIG_NSH_DISABLE_CH4_AS830)
extern void Ch4As830(void);
int cmd_As830(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    Ch4As830();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_Co2Zg09
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_CO2_ZG09) && !defined(CONFIG_NSH_DISABLE_CO2ZG09)
extern void Co2Zg09(void);
int cmd_Co2Zg09(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    Co2Zg09();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_Pm10Ps5308
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_PM1_0_PS5308) && !defined(CONFIG_NSH_DISABLE_PM1_0PS5308)
extern void Pm10Ps5308(void);
int cmd_Pm10Ps5308(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    Pm10Ps5308();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_Pm25Ps5308
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_PM2_5_PS5308) && !defined(CONFIG_NSH_DISABLE_PM2_5PS5308)
extern void Pm25Ps5308(void);
int cmd_Pm25Ps5308(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    Pm25Ps5308();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_Pm10Ps5308
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_PM10_PS5308) && !defined(CONFIG_NSH_DISABLE_PM10PS5308)
extern void Pm100Ps5308(void);
int cmd_Pm100Ps5308(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    Pm100Ps5308();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_VoiceD124
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_VOICE_D124) && !defined(CONFIG_NSH_DISABLE_VOICED124)
extern void VoiceD124(void);
int cmd_VoiceD124(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    VoiceD124();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_TempHs300x
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_TEMPERATURE_HS300X) && !defined(CONFIG_NSH_DISABLE_TEMPHS300X)
extern void TempHs300x(void);
int cmd_TempHs300x(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    TempHs300x();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_HumiHs300x
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_HUMIDITY_HS300X) && !defined(CONFIG_NSH_DISABLE_HUMIHS300X)
extern void HumiHs300x(void);
int cmd_HumiHs300x(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    HumiHs300x();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_WindDirectionQsFx
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_WINDDIRECTION_QS_FX) && !defined(CONFIG_NSH_DISABLE_QS_FX)
extern void WindDirectionQsFx(void);
int cmd_WindDirectionQsFx(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    WindDirectionQsFx();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_WindSpeedQsFs
 ****************************************************************************/

#if defined(CONFIG_APPLICATION_SENSOR_WINDSPEED_QS_FS) && !defined(CONFIG_NSH_DISABLE_QS_FS)
extern void WindSpeedQsFs(void);
int cmd_WindSpeedQsFs(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    WindSpeedQsFs();
    return OK;
}
#endif

/****************************************************************************
 * Name: cmd_openzigbee
 ****************************************************************************/

#if defined(CONFIG_CONNECTION_ADAPTER_ZIGBEE) && !defined(CONFIG_NSH_DISABLE_OPENZIGBEE)
extern int openzigbee(void);
int cmd_openzigbee(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    openzigbee();
    return OK;
}
#endif


#if defined(CONFIG_CONNECTION_ADAPTER_ZIGBEE) && !defined(CONFIG_NSH_DISABLE_SENDZIGBEE)
extern int sendzigbee(int argc, char *argv[]);
int cmd_sendzigbee(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    sendzigbee(argc,argv);
    return OK;
}
#endif

#if defined(CONFIG_CONNECTION_ADAPTER_ZIGBEE) && !defined(CONFIG_NSH_DISABLE_RECVZIGBEE)
extern int recvzigbee(void);
int cmd_recvzigbee(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    recvzigbee();
    return OK;
}
#endif

#if defined(CONFIG_ADAPTER_SX1278) && !defined(CONFIG_NSH_DISABLE_ADAPTER_LORATEST)
extern int AdapterLoraTest(void);
int cmd_AdapterLoraTest(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    AdapterLoraTest();
    return OK;
}
#endif

#if defined(CONFIG_ADAPTER_4G_EC200T) && !defined(CONFIG_NSH_DISABLE_ADAPTER_4GTEST)
extern int Adapter4GTest(void);
int cmd_Adapter4GTest(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, world!\n");
    FrameworkInit();
    Adapter4GTest();
    return OK;
}
#endif

#if defined(CONFIG_K210_FFT_TEST) && !defined(CONFIG_NSH_DISABLE_K210_FFT)
extern void nuttx_k210_fft_test(void);
int cmd_fft(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    nsh_output(vtbl, "Hello, fft!\n");
    nuttx_k210_fft_test();
    return OK;
}
#endif
