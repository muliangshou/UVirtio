#ifndef _JSON_PARSER_H_
#define _JSON_PARSER_H_

#define ANCHOR_NUM 5
#define JSON_BUFFER_SIZE (4 * 1024)

// params from json
typedef struct {
    float anchor[ANCHOR_NUM * 2];
    int net_output_shape[3];
    int net_input_size[2];
    int sensor_output_size[2];
    int kmodel_size;
    float obj_thresh[20];
    float nms_thresh;
    char labels[20][32];
    int class_num;
    int is_valid;
} yolov2_params_t;

yolov2_params_t param_parse(char *json_file_path);

#endif
