
export KERNELPATHS:=

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/xidatong)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/inc \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/port \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host/class \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/osa \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/phy \
	-I$(BSP_ROOT)/third_party_driver/ethernet \
	-I$(BSP_ROOT)/third_party_driver/ethernet/lan8720 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052/drivers \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/Include \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch \
	-I$(KERNEL_ROOT)/resources/include \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/xip #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/kd233)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/include \
    -I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/maix-go)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/include \
    -I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32f407-st-discovery)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/sdio \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/aiit-arm32-board)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/src/radio \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32f407zgt6)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/sdio\
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/gapuino)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/gd32vf103_rvstar)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/risc-v/gd32vf103_rvstar \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/rv32m1_vega)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/risc-v/rv32m1_vega \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/hifive1-rev-B)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/sifive \
	-I$(BSP_ROOT)/third_party_driver/include/sifive/devices \
	-I$(BSP_ROOT)/third_party_driver/fe300prci \
	-I$(BSP_ROOT)/third_party_driver/plic\
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/hifive1-emulator)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/sifive \
	-I$(BSP_ROOT)/third_party_driver/include/sifive/devices \
	-I$(BSP_ROOT)/third_party_driver/fe300prci \
	-I$(BSP_ROOT)/third_party_driver/plic\
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/k210-emulator)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/src/radio \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/aiit-riscv64-board)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/src/radio \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/cortex-m0-emulator)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m0 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/Libraries/driverlib \
	-I$(BSP_ROOT)/third_party_driver/Libraries/ \
	-I$(BSP_ROOT)/third_party_driver/Libraries/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/cortex-m3-emulator)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m3 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/Libraries/driverlib \
	-I$(BSP_ROOT)/third_party_driver/Libraries/ \
	-I$(BSP_ROOT)/third_party_driver/Libraries/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/cortex-m4-emulator)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/sdio\
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/ok1052-c)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/inc \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/port \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host/class \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/osa \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/phy \
	-I$(BSP_ROOT)/third_party_driver/ethernet \
	-I$(BSP_ROOT)/third_party_driver/ethernet/ksz8081 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052/drivers \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/Include \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch \
	-I$(KERNEL_ROOT)/resources/include \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/xip #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32f103-nano)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m3 \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/libraries/STM32F1xx_HAL/inc \
	-I$(BSP_ROOT)/third_party_driver/libraries/CMSIS \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/nuvoton-m2354)
KERNELPATHS :=-I$(BSP_ROOT) \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m23 \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/Drivers \
	-I$(BSP_ROOT)/third_party_driver/include/Regs \
	-I$(BSP_ROOT)/third_party_driver/include/NuUtils/inc \
	-I$(BSP_ROOT)/third_party_driver/common/NuClockConfig \
	-I$(BSP_ROOT)/third_party_driver/common/NuPinConfig \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

KERNELPATHS += -I$(KERNEL_ROOT)/arch \
            -I$(KERNEL_ROOT)/arch/risc-v/shared/kernel_service #

ifeq ($(CONFIG_FS_VFS), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/fs/devfs \
	-I$(KERNEL_ROOT)/fs/fatfs \
	-I$(KERNEL_ROOT)/fs/shared/include #
endif

ifeq ($(CONFIG_FS_CH376), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/fs/compatibility_ch376 #
endif

ifeq ($(CONFIG_TRANSFORM_LAYER_ATTRIUBUTE), y)
ifeq ($(CONFIG_ADD_XIZI_FETURES), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/transform_layer/xizi/user_api/posix_support/include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/transform_layer/xizi #
endif

ifeq ($(CONFIG_ADD_NUTTX_FETURES), y)
#
endif

ifeq ($(CONFIG_ADD_RTTHREAD_FETURES), y)
#
endif

endif

KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Applications/general_functions/list #

ifeq ($(CONFIG_SUPPORT_SENSOR_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/sensor #
endif

ifeq ($(CONFIG_SUPPORT_CONNECTION_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/zigbee #
endif

ifeq ($(CONFIG_ADAPTER_HFA21_ETHERCAT), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/industrial_ethernet/ethercat #
endif

ifeq ($(CONFIG_SUPPORT_KNOWING_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/gemmlowp #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/flatbuffers/include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/ruy #
endif

ifeq ($(CONFIG_LIB_LV),y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lvgl #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lvgl/examples/porting #
endif

ifeq ($(CONFIG_SUPPORT_CONTROL_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/shared #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc/interoperability/opcua #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc/shared #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/cJSON
endif

ifeq ($(CONFIG_CRYPTO), y)
KERNELPATHS += -I$(KERNEL_ROOT)/framework/security/crypto/include #
endif

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include #

ifeq ($(CONFIG_RESOURCES_SPI), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/resources/spi #

ifeq ($(CONFIG_RESOURCES_SPI_SFUD),y)
KERNELPATHS += -I$(KERNEL_ROOT)/resources/spi/third_party_spi/SFUD/sfud/inc #
endif

endif

ifeq ($(CONFIG_TOOL_SHELL), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/tool/shell/letter-shell \
	-I$(KERNEL_ROOT)/tool/shell/letter-shell/file_ext #
endif


ifeq ($(CONFIG_LIB_NEWLIB),y)
KERNELPATHS += -I$(KERNEL_ROOT)/lib/newlib/include #
endif

ifeq ($(CONFIG_FS_LWEXT4),y)
KERNELPATHS += -I$(KERNEL_ROOT)/fs/lwext4/lwext4_submodule/blockdev/xiuos #
KERNELPATHS += -I$(KERNEL_ROOT)/fs/lwext4/lwext4_submodule/include #
KERNELPATHS += -I$(KERNEL_ROOT)/fs/lwext4/lwext4_submodule/include/misc 
endif


ifeq ($(ARCH), risc-v)
KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/shared
ifeq ($(MCU), k210)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/k210
endif
ifeq ($(MCU), FE310)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/fe310
endif
ifeq ($(MCU), GAP8)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/gap8
endif
ifeq ($(MCU), GD32VF103)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/gd32vf103
endif
endif


ifeq ($(ARCH), arm)
KERNELPATHS +=-I$(KERNEL_ROOT)/arch/arm/shared \
	-I$(KERNEL_ROOT)/lib/comlibc/common #
endif


KERNELPATHS += -I$(KERNEL_ROOT)/kernel/include #


COMPILE_KERNEL:
	@$(eval CPPPATHS=$(KERNELPATHS))
	@for dir in $(SRC_KERNEL_DIR);do    \
               $(MAKE) -C $$dir;          \
       done
	@cp link.mk build/Makefile
	@$(MAKE) -C build COMPILE_TYPE="_kernel" TARGET=XiZi_$(BOARD)_kernel.elf LINK_FLAGS=LFLAGS
	@rm build/Makefile build/make.obj

