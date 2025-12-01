# SRC_FILES += lv_font.c
# SRC_FILES += lv_font_fmt_txt.c
# SRC_FILES += lv_font_loader.c

# SRC_FILES += lv_font_dejavu_16_persian_hebrew.c
# SRC_FILES += lv_font_montserrat_8.c
# SRC_FILES += lv_font_montserrat_10.c
# SRC_FILES += lv_font_montserrat_12.c
# SRC_FILES += lv_font_montserrat_12_subpx.c
# SRC_FILES += lv_font_montserrat_14.c
# SRC_FILES += lv_font_montserrat_16.c
# SRC_FILES += lv_font_montserrat_18.c
# SRC_FILES += lv_font_montserrat_20.c
# SRC_FILES += lv_font_montserrat_22.c
# SRC_FILES += lv_font_montserrat_24.c
# SRC_FILES += lv_font_montserrat_26.c
# SRC_FILES += lv_font_montserrat_28.c
# SRC_FILES += lv_font_montserrat_28_compressed.c
# SRC_FILES += lv_font_montserrat_30.c
# SRC_FILES += lv_font_montserrat_32.c
# SRC_FILES += lv_font_montserrat_34.c
# SRC_FILES += lv_font_montserrat_36.c
# SRC_FILES += lv_font_montserrat_38.c
# SRC_FILES += lv_font_montserrat_40.c
# SRC_FILES += lv_font_montserrat_42.c
# SRC_FILES += lv_font_montserrat_44.c
# SRC_FILES += lv_font_montserrat_46.c
# SRC_FILES += lv_font_montserrat_48.c
# SRC_FILES += lv_font_simsun_16_cjk.c
# SRC_FILES += lv_font_unscii_8.c
# SRC_FILES += lv_font_unscii_16.c

# DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/font
# VPATH += :$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/font

# CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/font"
SRC_FILES += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/font -name \*.c)

