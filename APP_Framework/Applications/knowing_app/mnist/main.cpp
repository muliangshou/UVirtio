#include <cstdio>
#include <transform.h>
#include <stdio.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "digit.h"
#include "model.h"

namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
constexpr int kTensorArenaSize = 110 * 1024;
uint8_t *tensor_arena = nullptr;
//uint8_t tensor_arena[kTensorArenaSize];
}

extern "C" void mnist_app() {
  tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  model = tflite::GetModel(mnist_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  tensor_arena = (uint8_t *)malloc(kTensorArenaSize);
  if (tensor_arena == nullptr) {
    TF_LITE_REPORT_ERROR(error_reporter, "malloc for tensor_arena failed");
    return;
  }

  tflite::AllOpsResolver resolver;
  tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  printf("------- Input Digit -------\n");
  for (int i = 0; i < 28; i++) {
    for (int j = 0; j < 28; j++) {
      if (mnist_digit[i*28+j] > 0.3)
        printf("#");
      else
        printf(".");
    }
    printf("\n");
  }

  for (int i = 0; i < 28*28; i++) {
        input->data.f[i] = mnist_digit[i];
    }

  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed on x_val\n");
    return;
  }

  // Read the predicted y value from the model's output tensor
  float max = 0.0;
  int index;
  for (int i = 0; i < 10; i++) {
         if(output->data.f[i]>max){
           max = output->data.f[i];
           index = i;
         }
  }
  printf("------- Output Result -------\n");
  printf("result is %d\n", index);
}

extern "C" {
#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(mnist_app, run mnist app);
#endif
}
