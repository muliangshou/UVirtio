include $(KERNEL_ROOT)/.config

ifeq ($(CONFIG_ADD_NUTTX_FETURES),y)
    include $(APPDIR)/Make.defs
 
    ifeq ($(CONFIG_K210_FFT_TEST), y)
        CSRCS += fft_soft.c fft_test.c
    endif

    include $(APPDIR)/Application.mk

endif
