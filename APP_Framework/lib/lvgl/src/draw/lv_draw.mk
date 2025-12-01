# SRC_FILES += lv_draw_arc.c
# SRC_FILES += lv_draw_blend.c
# SRC_FILES += lv_draw_img.c
# SRC_FILES += lv_draw_label.c
# SRC_FILES += lv_draw_line.c
# SRC_FILES += lv_draw_mask.c
# SRC_FILES += lv_draw_rect.c
# SRC_FILES += lv_draw_triangle.c
# SRC_FILES += lv_img_buf.c
# SRC_FILES += lv_img_cache.c
# SRC_FILES += lv_img_decoder.c

# DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/draw
# VPATH += :$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/draw

# CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/draw"
SRC_FILES += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/draw -name \*.c)

