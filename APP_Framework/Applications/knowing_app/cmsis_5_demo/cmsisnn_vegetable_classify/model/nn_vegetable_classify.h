#ifndef __NN_H__
#define __NN_H__

#include <transform.h>
#include "arm_math.h"
#include "arm_nnfunctions.h"
#include "parameter_vegetable_classify.h"
#include "weights_vegetable_classify.h"

void run_nn_detection(q7_t* input_data, q7_t* output_data);
void memory_init();

#endif
