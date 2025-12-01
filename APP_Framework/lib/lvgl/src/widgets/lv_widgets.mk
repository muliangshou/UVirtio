# SRC_FILES += lv_arc.c
# SRC_FILES += lv_bar.c
# SRC_FILES += lv_btn.c
# SRC_FILES += lv_btnmatrix.c
# SRC_FILES += lv_canvas.c
# SRC_FILES += lv_checkbox.c
# SRC_FILES += lv_dropdown.c
# SRC_FILES += lv_img.c
# SRC_FILES += lv_label.c
# SRC_FILES += lv_line.c
# SRC_FILES += lv_roller.c
# SRC_FILES += lv_slider.c
# SRC_FILES += lv_switch.c
# SRC_FILES += lv_table.c
# SRC_FILES += lv_textarea.c

# DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/widgets
# VPATH += :$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/widgets

# CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/widgets"
SRC_FILES += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/widgets -name \*.c)

