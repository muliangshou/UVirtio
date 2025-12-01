#include <string.h>
#include <transform.h>

#define FEATURE_NUM 4
#define CSV_PATH "/csv/iris.csv"
#define CSV_BUFFER_SIZE (1 * 1024)

static float data[10][FEATURE_NUM] = {};
static int data_len = 0;

void simple_CSV_read()
{
    int fin;
    int col = 0;
    char buffer[CSV_BUFFER_SIZE] = "";
    char *tmp = "";
    char *delim = ",\n ";

    fin = open(CSV_PATH, O_RDONLY);
    if (!fin) {
        printf("Error open file %s", CSV_PATH);
        // exit(-1);
        return;
    }
    read(fin, buffer, sizeof(buffer));
    close(fin);

    data_len = 0;
    for (tmp = strtok(buffer, delim); tmp && *tmp; col++, tmp = strtok(NULL, delim)) {
        if (0 == col % FEATURE_NUM) {
            // printf("\n");
            data_len++;
            col = 0;
        }
        data[data_len - 1][col] = atof(tmp);
        // printf("%.4f ", data[data_len - 1][col]);
    }
    // printf("\n");
}

void iris_SVC_predict()
{
#include "SVCModel.h"
    int result;

    simple_CSV_read();

    for (int i = 0; i < data_len; i++) {
        result = predict(data[i]);
        printf("data %d: ", i + 1);
        for (int j = 0; j < FEATURE_NUM; j++) {
            printf("%.4f ", data[i][j]);
        }
        printf("result: %d\n", result);
    }
}
#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(iris_SVC_predict, iris predict by SVC);
#endif

void iris_DecisonTree_predict()
{
#include "DecisionTreeClassifierModel.h"
    int result;

    simple_CSV_read();

    for (int i = 0; i < data_len; i++) {
        result = predict(data[i]);
        printf("data %d: ", i + 1);
        for (int j = 0; j < FEATURE_NUM; j++) {
            printf("%.4f ", data[i][j]);
        }
        printf("result: %d\n", result);
    }
}
#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(iris_DecisonTree_predict, iris predict by decison tree classifier);
#endif

void iris_LogisticRegression_predict()
{
#include "LogisticRegressionModel.h"
    int result;

    simple_CSV_read();

    for (int i = 0; i < data_len; i++) {
        result = predict(data[i]);
        printf("data %d: ", i + 1);
        for (int j = 0; j < FEATURE_NUM; j++) {
            printf("%.4f ", data[i][j]);
        }
        printf("result: %d\n", result);
    }
}
#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(iris_LogisticRegression_predict, iris predict by logistic regression);
#endif
