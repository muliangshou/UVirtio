include $(KERNEL_ROOT)/.config

ifeq ($(CONFIG_ADD_NUTTX_FETURES),y)
    include $(APPDIR)/Make.defs
 
    ifeq ($(CONFIG_APPLICATION_SENSOR_HCHO_TB600B_WQ_HCHO1OS), y)
        CSRCS += hcho_tb600b_wq_hcho1os.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_TVOC_TB600B_TVOC10), y)
        CSRCS += tvoc_tb600b_tvoc10.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_IAQ_TB600B_IAQ10), y)
        CSRCS += iaq_tb600b_iaq10.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_CH4_AS830), y)
        CSRCS += ch4_as830.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_CO2_ZG09), y)
        CSRCS += co2_zg09.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_PM1_0_PS5308), y)
        CSRCS += pm1_0_ps5308.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_PM2_5_PS5308), y)
        CSRCS += pm2_5_ps5308.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_PM10_PS5308), y)
        CSRCS += pm10_0_ps5308.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_VOICE_D124), y)
        CSRCS += voice_d124.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_HUMIDITY_HS300X), y)
        CSRCS += humidity_hs300x.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_TEMPERATURE_HS300X), y)
        CSRCS += temperature_hs300x.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_WINDDIRECTION_QS_FX), y)
        CSRCS += winddirection_qs_fx.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_WINDSPEED_QS_FS), y)
        CSRCS += windspeed_qs_fs.c
    endif

    include $(APPDIR)/Application.mk

endif

ifeq ($(CONFIG_ADD_XIZI_FETURES),y)
    SRC_FILES := 

    ifeq ($(CONFIG_APPLICATION_SENSOR_HCHO_TB600B_WQ_HCHO1OS), y)
        SRC_FILES += hcho_tb600b_wq_hcho1os.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_TVOC_TB600B_TVOC10), y)
        SRC_FILES += tvoc_tb600b_tvoc10.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_IAQ_TB600B_IAQ10), y)
        SRC_FILES += iaq_tb600b_iaq10.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_CH4_AS830), y)
        SRC_FILES += ch4_as830.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_CO2_ZG09), y)
        SRC_FILES += co2_zg09.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_CO2_G8S), y)
        SRC_FILES += co2_g8s.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_PM1_0_PS5308), y)
        SRC_FILES += pm1_0_ps5308.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_PM2_5_PS5308), y)
        SRC_FILES += pm2_5_ps5308.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_PM10_PS5308), y)
        SRC_FILES += pm10_0_ps5308.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_VOICE_D124), y)
        SRC_FILES += voice_d124.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_HUMIDITY_HS300X), y)
        SRC_FILES += humidity_hs300x.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_TEMPERATURE_HS300X), y)
        SRC_FILES += temperature_hs300x.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_WINDDIRECTION_QS_FX), y)
        SRC_FILES += winddirection_qs_fx.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_WINDSPEED_QS_FS), y)
        SRC_FILES += windspeed_qs_fs.c
    endif

    ifeq ($(CONFIG_APPLICATION_SENSOR_ALTITUDE_BMP180), y)
        SRC_FILES += altitude_bmp180.c
    endif

    include $(KERNEL_ROOT)/compiler.mk

endif
