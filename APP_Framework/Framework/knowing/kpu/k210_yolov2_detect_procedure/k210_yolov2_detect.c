#include "k210_yolov2_detect.h"

#include "cJSON.h"
#ifdef USING_YOLOV2_JSONPARSER
#include <json_parser.h>
#endif
#include "region_layer.h"
#define STACK_SIZE (128 * 1024)

static dmac_channel_number_t dma_ch = DMAC_CHANNEL_MAX;

#define THREAD_PRIORITY_D (11)
static pthread_t tid = 0;
static void *thread_detect_entry(void *parameter);
static int g_fd = 0;
static int kmodel_fd = 0;
static int if_exit = 0;
static unsigned char *showbuffer = NULL;
static unsigned char *kpurgbbuffer = NULL;

static _ioctl_shoot_para shoot_para_t = {0};
unsigned char *model_data = NULL;  // kpu data  load memory
unsigned char *model_data_align = NULL;

kpu_model_context_t detect_task;
static region_layer_t detect_rl;
static obj_info_t detect_info;
volatile uint32_t g_ai_done_flag;

static void ai_done(void *ctx) { g_ai_done_flag = 1; }

void k210_detect(char *json_file_path)
{
    int ret = 0;
    int result = 0;
    int size = 0;
    char kmodel_path[127] = {};

    yolov2_params_t detect_params = param_parse(json_file_path);
    if (!detect_params.is_valid) {
        return;
    }

    g_fd = open("/dev/ov2640", O_RDONLY);
    if (g_fd < 0) {
        printf("open ov2640 fail !!");
        return;
    }
    _ioctl_set_reso set_dvp_reso = {detect_params.sensor_output_size[1], detect_params.sensor_output_size[0]};
    ioctl(g_fd, IOCTRL_CAMERA_OUT_SIZE_RESO, &set_dvp_reso);
    showbuffer =
        (unsigned char *)rt_malloc_align(detect_params.sensor_output_size[0] * detect_params.sensor_output_size[1] * 2, 64);
    if (NULL == showbuffer) {
        close(g_fd);
        printf("showbuffer apply memory fail !!");
        return;
    }
    kpurgbbuffer = (unsigned char *)rt_malloc_align(detect_params.net_input_size[0] * detect_params.net_input_size[1] * 3, 64);
    if (NULL == kpurgbbuffer) {
        close(g_fd);
        rt_free_align(showbuffer);
        printf("kpurgbbuffer apply memory fail !!");
        return;
    }
    model_data = (unsigned char *)malloc(detect_params.kmodel_size + 255);
    if (NULL == model_data) {
        rt_free_align(showbuffer);
        rt_free_align(kpurgbbuffer);
        close(g_fd);
        printf("model_data apply memory fail !!");
        return;
    }
    memset(model_data, 0, detect_params.kmodel_size + 255);
    memset(showbuffer, 0, detect_params.sensor_output_size[0] * detect_params.sensor_output_size[1] * 2);
    memset(kpurgbbuffer, 0, detect_params.net_input_size[0] * detect_params.net_input_size[1] * 3);
    shoot_para_t.pdata = (unsigned int *)(showbuffer);
    shoot_para_t.length = (size_t)(detect_params.sensor_output_size[0] * detect_params.sensor_output_size[1] * 2);
    /*
        load memory
    */
    // kmodel path generate from json file path, *.json -> *.kmodel
    memcpy(kmodel_path, json_file_path, strlen(json_file_path));
    int idx_suffix_start = strlen(json_file_path) - 4;
    const char kmodel_suffix[7] = "kmodel";
    int kmodel_suffix_len = 6;
    while (kmodel_suffix_len--) {
        kmodel_path[idx_suffix_start + 5 - kmodel_suffix_len] = kmodel_suffix[5 - kmodel_suffix_len];
    }
    printf("kmodel path: %s\n", kmodel_path);
    kmodel_fd = open(kmodel_path, O_RDONLY);
    if (kmodel_fd < 0) {
        printf("open kmodel fail");
        close(g_fd);
        free(showbuffer);
        free(kpurgbbuffer);
        free(model_data);
        return;
    } else {
        size = read(kmodel_fd, model_data, detect_params.kmodel_size);
        if (size != detect_params.kmodel_size) {
            printf("read kmodel error size %d\n", size);
            close(g_fd);
            close(kmodel_fd);
            free(showbuffer);
            free(kpurgbbuffer);
            free(model_data);
            return;

        } else {
            printf("read kmodel success \n");
        }
    }
    unsigned char *model_data_align = (unsigned char *)(((unsigned int)model_data + 255) & (~255));
    // dvp_set_ai_addr((uint32_t)kpurgbbuffer,
    //                 (uint32_t)(kpurgbbuffer + detect_params.net_input_size[0] * detect_params.net_input_size[1]),
    //                 (uint32_t)(kpurgbbuffer + detect_params.net_input_size[0] * detect_params.net_input_size[1] * 2));
    dvp_set_ai_addr(
        (uint32_t)(kpurgbbuffer +
                   detect_params.net_input_size[1] * (detect_params.net_input_size[0] - detect_params.sensor_output_size[0])),
        (uint32_t)(kpurgbbuffer +
                   detect_params.net_input_size[1] * (detect_params.net_input_size[0] - detect_params.sensor_output_size[0]) +
                   detect_params.net_input_size[0] * detect_params.net_input_size[1]),
        (uint32_t)(kpurgbbuffer +
                   detect_params.net_input_size[1] * (detect_params.net_input_size[0] - detect_params.sensor_output_size[0]) +
                   detect_params.net_input_size[0] * detect_params.net_input_size[1] * 2));
    if (kpu_load_kmodel(&detect_task, model_data_align) != 0) {
        printf("\nmodel init error\n");
        close(g_fd);
        close(kmodel_fd);
        free(showbuffer);
        free(kpurgbbuffer);
        free(model_data);
        return;
    }

    detect_rl.anchor_number = ANCHOR_NUM;
    detect_rl.anchor = detect_params.anchor;
    detect_rl.nms_value = detect_params.nms_thresh;
    detect_rl.classes = detect_params.class_num;
    result =
        region_layer_init(&detect_rl, detect_params.net_output_shape[0], detect_params.net_output_shape[1],
                          detect_params.net_output_shape[2], detect_params.net_input_size[1], detect_params.net_input_size[0]);
    printf("region_layer_init result %d \n\r", result);
    for (int idx = 0; idx < detect_params.class_num; idx++) {
        detect_rl.threshold[idx] = detect_params.obj_thresh[idx];
    }

    size_t stack_size = STACK_SIZE;
    pthread_attr_t attr;                      /* 线程属性 */
    struct sched_param prio;                  /* 线程优先级 */
    prio.sched_priority = 8;                  /* 优先级设置为 8 */
    pthread_attr_init(&attr);                 /* 先使用默认值初始化属性 */
    pthread_attr_setschedparam(&attr, &prio); /* 修改属性对应的优先级 */
    pthread_attr_setstacksize(&attr, stack_size);

    /* 创建线程 1, 属性为 attr，入口函数是 thread_entry，入口函数参数是 1 */
    result = pthread_create(&tid, &attr, thread_detect_entry, &detect_params);
    if (0 == result) {
        printf("thread_detect_entry successfully!\n");
    } else {
        printf("thread_detect_entry failed! error code is %d\n", result);
        close(g_fd);
    }
}
// #ifdef __RT_THREAD_H__
// MSH_CMD_EXPORT(detect, detect task);
// #endif

static void *thread_detect_entry(void *parameter)
{
    yolov2_params_t detect_params = *(yolov2_params_t *)parameter;
    extern void lcd_draw_picture(uint16_t x1, uint16_t y1, uint16_t width, uint16_t height, uint32_t * ptr);
    printf("thread_detect_entry start!\n");
    int ret = 0;
    // sysctl_enable_irq();
    while (1) {
        // memset(showbuffer,0,320*240*2);
        g_ai_done_flag = 0;
        ret = ioctl(g_fd, IOCTRL_CAMERA_START_SHOT, &shoot_para_t);
        if (RT_ERROR == ret) {
            printf("ov2640 can't wait event flag");
            rt_free(showbuffer);
            close(g_fd);
            pthread_exit(NULL);
            return NULL;
        }
        if (dmalock_sync_take(&dma_ch, 2000)) {
            printf("Fail to take DMA channel");
        }
        kpu_run_kmodel(&detect_task, kpurgbbuffer, DMAC_CHANNEL5, ai_done, NULL);
        while (!g_ai_done_flag)
            ;
        dmalock_release(dma_ch);
        float *output;
        size_t output_size;
        kpu_get_output(&detect_task, 0, (uint8_t **)&output, &output_size);
        detect_rl.input = output;
        region_layer_run(&detect_rl, &detect_info);
        /* display result */

        for (int cnt = 0; cnt < detect_info.obj_number; cnt++) {
            detect_info.obj[cnt].y1 += (detect_params.sensor_output_size[0] - detect_params.net_input_size[0])/2;
            detect_info.obj[cnt].y2 += (detect_params.sensor_output_size[0] - detect_params.net_input_size[0])/2;
            draw_edge((uint32_t *)showbuffer, &detect_info, cnt, 0xF800, (uint16_t)detect_params.sensor_output_size[1],
                      (uint16_t)detect_params.sensor_output_size[0]);
            printf("%d: (%d, %d, %d, %d) cls: %s conf: %f\t", cnt, detect_info.obj[cnt].x1, detect_info.obj[cnt].y1,
                   detect_info.obj[cnt].x2, detect_info.obj[cnt].y2, detect_params.labels[detect_info.obj[cnt].class_id],
                   detect_info.obj[cnt].prob);
        }
#ifdef BSP_USING_LCD
        lcd_draw_picture(0, 0, (uint16_t)detect_params.sensor_output_size[1] - 1,
                         (uint16_t)detect_params.sensor_output_size[0] - 1, (uint32_t *)showbuffer);
        // lcd_show_image(0, 0, (uint16_t)detect_params.sensor_output_size[1], (uint16_t)detect_params.sensor_output_size[0],
        // (unsigned int *)showbuffer);
#endif
        usleep(500);
        if (1 == if_exit) {
            if_exit = 0;
            printf("thread_detect_entry exit");
            pthread_exit(NULL);
        }
    }
}

void detect_delete()
{
    if (showbuffer != NULL) {
        int ret = 0;
        close(g_fd);
        close(kmodel_fd);
        free(showbuffer);
        free(kpurgbbuffer);
        free(model_data);
        printf("detect task cancel!!! ret %d ", ret);
        if_exit = 1;
    }
}
// #ifdef __RT_THREAD_H__
// MSH_CMD_EXPORT(detect_delete, detect task delete);
// #endif

// void kmodel_load(unsigned char *model_data)
// {
//     int kmodel_fd = 0;
//     int size = 0;
//     char kmodel_path[127] = {};
//     // kmodel path generate from json file path, *.json -> *.kmodel
//     memcpy(kmodel_path, json_file_path, strlen(json_file_path));
//     int idx_suffix_start = strlen(json_file_path) - 4;
//     const char kmodel_suffix[5] = "kmodel";
//     int kmodel_suffix_len = 5;
//     while (kmodel_suffix_len--) {
//         kmodel_path[idx_suffix_start + 4 - kmodel_suffix_len] = kmodel_suffix[4 - kmodel_suffix_len];
//     }
//     printf("Kmodel path: %s\n", kmodel_path);
//     kmodel_fd = open(kmodel_path, O_RDONLY);

//     model_data = (unsigned char *)malloc(detect_params.kmodel_size + 255);
//     if (NULL == model_data) {
//         printf("model_data apply memory fail !!");
//         return;
//     }
//     memset(model_data, 0, detect_params.kmodel_size + 255);

//     if (kmodel_fd >= 0) {
//         size = read(kmodel_fd, model_data, detect_params.kmodel_size);
//         if (size != detect_params.kmodel_size) {
//             printf("read kmodel error size %d\n", size);

//         } else {
//             printf("read kmodel success");
//         }
//     } else {
//         free(model_data);
//         printf("open kmodel fail");
//     }
// }
// #ifdef __RT_THREAD_H__
// MSH_CMD_EXPORT(kmodel_load, kmodel load memory);
// #endif
