# SRC_FILES += lv_gpu_nxp_pxp.c
# SRC_FILES += lv_gpu_nxp_pxp_osa.c
# SRC_FILES += lv_gpu_nxp_vglite.c
# SRC_FILES += lv_gpu_stm32_dma2d.c
# SRC_FILES += lv_gpu_sdl.c

# DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/gpu
# VPATH += :$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/gpu

# CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/gpu"
SRC_FILES += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/gpu -name \*.c)


include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/gpu/sdl/lv_gpu_sdl.mk