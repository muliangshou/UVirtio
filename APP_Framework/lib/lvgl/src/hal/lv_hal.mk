# SRC_FILES += lv_hal_disp.c
# SRC_FILES += lv_hal_indev.c
# SRC_FILES += lv_hal_tick.c

# DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/hal
# VPATH += :$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/hal

# CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/hal"
SRC_FILES += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/hal -name \*.c)

