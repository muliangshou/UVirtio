SRC_FILES := 

ifeq ($(CONFIG_USER_TEST_SPI_FLASH),y)
	SRC_FILES += test_spi_flash.c
endif

ifeq ($(CONFIG_USER_TEST_ADC),y)
	SRC_FILES += test_adc.c
endif

ifeq ($(CONFIG_USER_TEST_DAC),y)
	SRC_FILES += test_dac.c
endif

include $(KERNEL_ROOT)/compiler.mk
