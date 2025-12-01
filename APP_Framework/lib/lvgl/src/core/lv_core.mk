# SRC_FILES += lv_disp.c
# SRC_FILES += lv_group.c
# SRC_FILES += lv_indev.c
# SRC_FILES += lv_indev_scroll.c
# SRC_FILES += lv_obj.c
# SRC_FILES += lv_obj_class.c
# SRC_FILES += lv_obj_draw.c
# SRC_FILES += lv_obj_pos.c
# SRC_FILES += lv_obj_scroll.c
# SRC_FILES += lv_obj_style.c
# SRC_FILES += lv_obj_style_gen.c
# SRC_FILES += lv_obj_tree.c
# SRC_FILES += lv_event.c
# SRC_FILES += lv_refr.c
# SRC_FILES += lv_theme.c

# DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/core
# VPATH += :$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/core

# CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/core"
SRC_FILES += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/core -name \*.c)

