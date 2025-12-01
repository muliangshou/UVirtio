#include <lvgl.h>
#define DBG_TAG    "LVGL"
#define DBG_LVL    DBG_INFO

#ifndef PKG_USING_LVGL_DISP_DEVICE
#include <lv_port_disp_template.h>
#endif

#ifndef PKG_USING_LVGL_INDEV_DEVICE
#include <lv_port_indev_template.h>
#endif
extern void lv_port_disp_init(void);
extern void lv_port_indev_init(void);
#if LV_USE_LOG && LV_LOG_PRINTF
static void lv_rt_log(const char *buf)
{
    printf(buf);
    printf("\n");
}
#endif

int lv_port_init(void)
{
#if LV_USE_LOG && LV_LOG_PRINTF
    lv_log_register_print_cb(lv_rt_log);
#endif
    lv_init();

#ifndef PKG_USING_LVGL_DISP_DEVICE
    lv_port_disp_init();
#endif

#ifndef PKG_USING_LVGL_INDEV_DEVICE
    lv_port_indev_init();
#endif

    return 0;
}

