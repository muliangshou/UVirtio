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
#include <stdio.h>
#include <string.h>
#include <transform.h>

extern int SensorFrameworkInit(void);
extern int AdapterFrameworkInit(void);

extern int Adapter4GInit(void);
extern int AdapterNbiotInit(void);
extern int AdapterBlueToothInit(void);
extern int AdapterWifiInit(void);
extern int AdapterEthernetInit(void);
extern int AdapterEthercatInit(void);
extern int AdapterZigbeeInit(void);
extern int AdapterLoraInit(void);

extern int D124VoiceInit(void);
extern int Hs300xTemperatureInit(void);
extern int Hs300xHumidityInit(void);
extern int Ps5308Pm1_0Init(void);
extern int Ps5308Pm2_5Init(void);
extern int Ps5308Pm10Init(void);
extern int Zg09Co2Init(void);
extern int As830Ch4Init(void);
extern int Tb600bIaq10IaqInit(void);
extern int Tb600bTvoc10TvocInit(void);
extern int Tb600bWqHcho1osInit(void);
extern int QsFxWindDirectionInit(void);
extern int QsFsWindSpeedInit(void);

extern int lv_port_init(void);

typedef int (*InitFunc)(void);
struct InitDesc
{
	const char* fn_name;
	const InitFunc fn;
};

static int AppInitDesc(struct InitDesc sub_desc[])
{
	int i = 0;
	int ret = 0;
	for( i = 0; sub_desc[i].fn != NULL; i++ ) {
		ret = sub_desc[i].fn();
		printf("initialize %s %s\n",sub_desc[i].fn_name, ret == 0 ? "success" : "failed");
		if(0 != ret) {
			break;
		}
	}
	return ret;
}

static struct InitDesc framework[] = 
{
#ifdef SUPPORT_SENSOR_FRAMEWORK
	{ "sensor_framework", SensorFrameworkInit },
#endif

#ifdef SUPPORT_CONNECTION_FRAMEWORK
	{ "connection_framework", AdapterFrameworkInit },
#endif

	{ "NULL", NULL },
};

static struct InitDesc sensor_desc[] = 
{
#ifdef SENSOR_DEVICE_D124
	{ "d124_voice", D124VoiceInit },
#endif

#ifdef SENSOR_DEVICE_HS300X
#ifdef SENSOR_QUANTITY_HS300X_TEMPERATURE
	{ "hs300x_temperature", Hs300xTemperatureInit },
#endif
#ifdef SENSOR_QUANTITY_HS300X_HUMIDITY
	{ "hs300x_humidity", Hs300xHumidityInit },
#endif
#endif

#ifdef SENSOR_PS5308
#ifdef SENSOR_QUANTITY_PS5308_PM1_0
	{ "ps5308_pm1_0", Ps5308Pm1_0Init },
#endif
#ifdef SENSOR_QUANTITY_PS5308_PM2_5
	{ "ps5308_pm2_5", Ps5308Pm2_5Init },
#endif
#ifdef SENSOR_QUANTITY_PS5308_PM10
	{ "ps5308_pm10", Ps5308Pm10Init },
#endif
#endif

#ifdef SENSOR_ZG09
	{ "zg09_co2", Zg09Co2Init },
#endif

#ifdef SENSOR_QS_FX
	{ "qs_fx_wind_direction", QsFxWindDirectionInit },
#endif

#ifdef SENSOR_QS_FS
	{ "qs_fs_wind_speed", QsFsWindSpeedInit },
#endif

#ifdef SENSOR_AS830
	{ "ch4_as830", As830Ch4Init },
#endif

#ifdef SENSOR_TB600B_IAQ10
	{ "iaq_tb600b_iaq10", Tb600bIaq10IaqInit },
#endif

#ifdef SENSOR_TB600B_TVOC10
	{ "tvoc_tb600b_tvoc10", Tb600bTvoc10TvocInit },
#endif

#ifdef SENSOR_TB600B_WQ_HCHO1OS
	{ "tvoc_tb600b_wq_hcho1os", Tb600bWqHcho1osInit },
#endif

	{ "NULL", NULL },
};

static struct InitDesc connection_desc[] = 
{
#ifdef CONNECTION_ADAPTER_4G
	{ "4G adapter", Adapter4GInit},
#endif
#ifdef CONNECTION_ADAPTER_NB
	{ "NB adpter", AdapterNbiotInit},
#endif
#ifdef CONNECTION_ADAPTER_ZIGBEE
	{ "zigbee adapter", AdapterZigbeeInit},
#endif
#ifdef CONNECTION_ADAPTER_BLUETOOTH
	{ "bluetooth adapter", AdapterBlueToothInit},
#endif
#ifdef CONNECTION_ADAPTER_WIFI
	{ "wifi adapter", AdapterWifiInit},
#endif
#ifdef CONNECTION_ADAPTER_ETHERNET
	{ "ethernet adapter", AdapterEthernetInit},
#endif
#ifdef CONNECTION_ADAPTER_ETHERCAT
	{ "ethercat adapter", AdapterEthercatInit},
#endif
#ifdef CONNECTION_ADAPTER_LORA
	{ "lora adapter", AdapterLoraInit},
#endif
	{ "NULL", NULL },
};

/**
 * This function will init sensor framework and all sub sensors
 * @param sub_desc framework
 * 
 */
static int SensorDeviceFrameworkInit(struct InitDesc sub_desc[])
{
	int i = 0;
	int ret = 0;
	for ( i = 0; sub_desc[i].fn != NULL; i++ ) {
		if (0 == strncmp(sub_desc[i].fn_name, "sensor_framework", strlen("sensor_framework"))) {
			ret = sub_desc[i].fn();
			break;
		}
	}

	if (0 == ret) {
		printf("initialize sensor_framework success.\n");
		AppInitDesc(sensor_desc);
	}

	return ret;
}

/**
 * This function will init connection framework and all sub components
 * @param sub_desc framework
 * 
 */
static int ConnectionDeviceFrameworkInit(struct InitDesc sub_desc[])
{
	int i = 0;
	int ret = 0;
	for ( i = 0; sub_desc[i].fn != NULL; i++ ) {
		if (0 == strncmp(sub_desc[i].fn_name, "connection_framework", strlen("connection_framework"))) {
			ret = sub_desc[i].fn();
			break;
		}
	}
	
	if (0 == ret) {
		printf("initialize connection_framework success.\n");
		AppInitDesc(connection_desc);
	}

	return ret;
}

/**
 * This function will init system framework
 * 
 */
int FrameworkInit(void)
{
#ifdef SUPPORT_SENSOR_FRAMEWORK
	SensorDeviceFrameworkInit(framework);
#endif

#ifdef SUPPORT_CONNECTION_FRAMEWORK
	ConnectionDeviceFrameworkInit(framework);
#endif

#ifdef LIB_LV
	lv_port_init();
#endif

    return 0;
}