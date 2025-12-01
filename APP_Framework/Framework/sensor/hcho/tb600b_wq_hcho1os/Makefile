include $(KERNEL_ROOT)/.config

ifeq ($(CONFIG_ADD_NUTTX_FETURES),y)
    include $(APPDIR)/Make.defs
    CSRCS += tb600b_wq_hcho1os.c
    include $(APPDIR)/Application.mk
endif

ifeq ($(CONFIG_ADD_XIZI_FETURES),y)
    SRC_FILES := tb600b_wq_hcho1os.c
    include $(KERNEL_ROOT)/compiler.mk
endif
