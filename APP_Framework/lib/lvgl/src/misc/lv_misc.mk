# SRC_FILES += lv_anim.c
# SRC_FILES += lv_anim_timeline.c
# SRC_FILES += lv_area.c
# SRC_FILES += lv_async.c
# SRC_FILES += lv_bidi.c
# SRC_FILES += lv_color.c
# SRC_FILES += lv_fs.c
# SRC_FILES += lv_gc.c
# SRC_FILES += lv_ll.c
# SRC_FILES += lv_log.c
# SRC_FILES += lv_math.c
# SRC_FILES += lv_mem.c
# SRC_FILES += lv_printf.c
# SRC_FILES += lv_style.c
# SRC_FILES += lv_style_gen.c
# SRC_FILES += lv_timer.c
# SRC_FILES += lv_tlsf.c
# SRC_FILES += lv_txt.c
# SRC_FILES += lv_txt_ap.c
# SRC_FILES += lv_utils.c

# DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/misc
# VPATH += :$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/misc

# CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/misc"
SRC_FILES += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/misc -name \*.c)

