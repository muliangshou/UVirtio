
export APPPATHS :=-I$(BSP_ROOT) \

APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Applications/general_functions/list \
	-I$(KERNEL_ROOT)/../../APP_Framework/lib/app_newlib/include \
	-I$(KERNEL_ROOT)/../../APP_Framework/Framework/sensor #

ifeq ($(CONFIG_ADD_XIZI_FETURES), y)
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/transform_layer/xizi \
	-I$(KERNEL_ROOT)/../../APP_Framework/Framework/transform_layer/xizi/user_api/switch_api \
	-I$(KERNEL_ROOT)/../../APP_Framework/Framework/transform_layer/xizi/user_api/posix_support/include #
endif

ifeq ($(CONFIG_SUPPORT_CONNECTION_FRAMEWORK), y)
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection #
endif

ifeq ($(CONFIG_SUPPORT_KNOWING_FRAMEWORK), y)
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing #
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source #
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/gemmlowp #
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/flatbuffers/include #
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/ruy #
endif

ifeq ($(CONFIG_SUPPORT_CONTROL_FRAMEWORK), y)
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control #
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc/interoperability/opcua #
endif

ifeq ($(CONFIG_CRYPTO), y)
APPPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/security/crypto/include #
endif

COMPILE_APP:
	@$(eval CPPPATHS=$(APPPATHS))
	@echo $(SRC_APP_DIR)
	@for dir in $(SRC_APP_DIR);do    \
               $(MAKE) -C $$dir;          \
       done
	@cp link.mk build/Makefile
	@$(MAKE) -C build COMPILE_TYPE="_app" TARGET=XiZi_$(BOARD)_app.elf LINK_FLAGS=APPLFLAGS
	@rm build/Makefile build/make.obj

