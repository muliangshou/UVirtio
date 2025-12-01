#include "json_parser.h"

#include <fcntl.h>

#include "cJSON.h"

yolov2_params_t param_parse(char *json_file_path)
{
    yolov2_params_t params_return;
    params_return.is_valid = 1;
    int fin;
    char buffer[JSON_BUFFER_SIZE] = "";
    // char *buffer;
    // if ((yolov2_params_t *)NULL != (buffer = (char*)malloc(JSON_BUFFER_SIZE * sizeof(char)))) {
    //     memset(buffer, 0, JSON_BUFFER_SIZE * sizeof(char));
    // } else {
    //     printf("Json buffer malloc failed!");
    //     params_return.is_valid = 0;
    //     return params_return;
    // }
    int array_size;
    cJSON *json_obj;
    cJSON *json_item;
    cJSON *json_array_item;

    fin = open(json_file_path, O_RDONLY);
    if (!fin) {
        printf("Error open file %s\n", json_file_path);
        params_return.is_valid = 0;
        return params_return;
    } else {
        printf("Reading config from: %s\n", json_file_path);
    }
    read(fin, buffer, sizeof(buffer));
    close(fin);

    // read json string
    json_obj = cJSON_Parse(buffer);
    // free(buffer);
    char *json_print_str = cJSON_Print(json_obj);
    printf("Json file content: \n%s\n", json_print_str);
    cJSON_free(json_print_str);
    // get anchors
    json_item = cJSON_GetObjectItem(json_obj, "anchors");
    array_size = cJSON_GetArraySize(json_item);
    if (ANCHOR_NUM * 2 != array_size) {
        printf("Expect anchor size: %d, got %d in json file", ANCHOR_NUM * 2, array_size);
        params_return.is_valid = 0;
        return params_return;
    } else {
        printf("Got %d anchors from json file\n", ANCHOR_NUM);
    }
    for (int i = 0; i < ANCHOR_NUM * 2; i++) {
        json_array_item = cJSON_GetArrayItem(json_item, i);
        params_return.anchor[i] = json_array_item->valuedouble;
        printf("%d: %f\n", i, params_return.anchor[i]);
    }
    // net_input_size
    json_item = cJSON_GetObjectItem(json_obj, "net_input_size");
    array_size = cJSON_GetArraySize(json_item);
    if (2 != array_size) {
        printf("Expect net_input_size: %d, got %d in json file", 2, array_size);
        params_return.is_valid = 0;
        return params_return;
    } else {
        printf("Got %d net_input_size from json file\n", 2);
    }
    for (int i = 0; i < 2; i++) {
        json_array_item = cJSON_GetArrayItem(json_item, i);
        params_return.net_input_size[i] = json_array_item->valueint;
        printf("%d: %d\n", i, params_return.net_input_size[i]);
    }
    // net_output_shape
    json_item = cJSON_GetObjectItem(json_obj, "net_output_shape");
    array_size = cJSON_GetArraySize(json_item);
    if (3 != array_size) {
        printf("Expect net_output_shape: %d, got %d in json file", 3, array_size);
        params_return.is_valid = 0;
        return params_return;
    } else {
        printf("Got %d net_output_shape from json file\n", 3);
    }
    for (int i = 0; i < 3; i++) {
        json_array_item = cJSON_GetArrayItem(json_item, i);
        params_return.net_output_shape[i] = json_array_item->valueint;
        printf("%d: %d\n", i, params_return.net_output_shape[i]);
    }
    // sensor_output_size
    json_item = cJSON_GetObjectItem(json_obj, "sensor_output_size");
    array_size = cJSON_GetArraySize(json_item);
    if (2 != array_size) {
        printf("Expect sensor_output_size: %d, got %d in json file", 2, array_size);
        params_return.is_valid = 0;
        return params_return;
    } else {
        printf("Got %d sensor_output_size from json file\n", 2);
    }
    for (int i = 0; i < 2; i++) {
        json_array_item = cJSON_GetArrayItem(json_item, i);
        params_return.sensor_output_size[i] = json_array_item->valueint;
        printf("%d: %d\n", i, params_return.sensor_output_size[i]);
    }
    // check sensor output width and net input width
    if (params_return.sensor_output_size[1] != params_return.net_input_size[1]) {
        printf("Net input width must match sensor output width!\n");
        params_return.is_valid = 0;
        return params_return;
    }
    // // kmodel_path
    // json_item = cJSON_GetObjectItem(json_obj, "kmodel_path");
    // memset(params_return.kmodel_path, 0, 127);
    // memcpy(params_return.kmodel_path, json_item->valuestring, strlen(json_item->valuestring));
    // printf("Got kmodel_path: %s\n", params_return.kmodel_path);
    // kmodel_size
    json_item = cJSON_GetObjectItem(json_obj, "kmodel_size");
    params_return.kmodel_size = json_item->valueint;
    printf("Got kmodel_size: %d\n", params_return.kmodel_size);
    // labels
    json_item = cJSON_GetObjectItem(json_obj, "labels");
    params_return.class_num = cJSON_GetArraySize(json_item);
    if (0 >= params_return.class_num) {
        printf("No labels!");
        params_return.is_valid = 0;
        return params_return;
    } else {
        printf("Got %d labels\n", params_return.class_num);
    }
    for (int i = 0; i < params_return.class_num; i++) {
        json_array_item = cJSON_GetArrayItem(json_item, i);
        memset(params_return.labels[i], 0, 127);
        memcpy(params_return.labels[i], json_array_item->valuestring, strlen(json_array_item->valuestring));
        printf("%d: %s\n", i, params_return.labels[i]);
    }
    // obj_thresh
    json_item = cJSON_GetObjectItem(json_obj, "obj_thresh");
    array_size = cJSON_GetArraySize(json_item);
    if (params_return.class_num != array_size) {
        printf("label number and thresh number mismatch! label number : %d, obj thresh number %d", params_return.class_num,
               array_size);
        params_return.is_valid = 0;
        return params_return;
    } else {
        printf("Got %d obj_thresh\n", array_size);
    }
    for (int i = 0; i < array_size; i++) {
        json_array_item = cJSON_GetArrayItem(json_item, i);
        params_return.obj_thresh[i] = json_array_item->valuedouble;
        printf("%d: %f\n", i, params_return.obj_thresh[i]);
    }
    // nms_thresh
    json_item = cJSON_GetObjectItem(json_obj, "nms_thresh");
    params_return.nms_thresh = json_item->valuedouble;
    printf("Got nms_thresh: %f\n", params_return.nms_thresh);

    cJSON_Delete(json_obj);
    return params_return;
}