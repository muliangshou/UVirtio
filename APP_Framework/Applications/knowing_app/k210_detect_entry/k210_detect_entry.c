#include "k210_yolov2_detect.h"
static void detect_app(int argc, char *argv[])
{
    if (2 != argc) {
        printf("Usage: detect_app <ABSOLUTE_CONFIG_JSON_PATH>");
    } else {
        k210_detect(argv[1]);
    }

    return;
}
// clang-format off
#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(detect_app, k210 detect app usage: detect_app <ABSOLUTE_CONFIG_JSON_PATH>);
#endif
// clang-format on
