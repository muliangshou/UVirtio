#include "nn_vegetable_classify.h"

static const q7_t conv1_w[CONV1_WT_SHAPE] = CONV1_WT;
static const q7_t conv1_b[CONV1_BIAS_SHAPE] = CONV1_BIAS;
static const q7_t conv2_w[CONV2_WT_SHAPE] = CONV2_WT;
static const q7_t conv2_b[CONV2_BIAS_SHAPE] = CONV2_BIAS;
// static const q7_t conv3_w[CONV3_WT_SHAPE] =  CONV3_WT;
// static const q7_t conv3_b[CONV3_BIAS_SHAPE] = CONV3_BIAS;
static const q7_t interface_w[INTERFACE_WT_SHAPE] = INTERFACE_WT;
static const q7_t interface_b[INTERFACE_BIAS_SHAPE] = INTERFACE_BIAS;
static const q7_t linear_w[LINEAR_WT_SHAPE] = LINEAR_WT;
static const q7_t linear_b[LINEAR_BIAS_SHAPE] = LINEAR_BIAS;

q7_t *conv1_out = NULL;
q7_t *pool1_out = NULL;
q7_t *conv2_out = NULL;
q7_t *pool2_out = NULL;
//  q7_t *conv3_out = NULL ;
q7_t *interface_out = NULL;
q7_t *linear_out = NULL;
q7_t *y_out = NULL;
q7_t *conv_buffer = NULL;
q7_t *fc_buffer = NULL;

void memory_init()
{
  static int flag = 0;
  if (flag == 0)
  {
    conv1_out = malloc(CONV1_OUT_CH * CONV1_OUT_DIM * CONV1_OUT_DIM);
    if (conv1_out == NULL)
    {
      printf("conv1_out malloc failed...\n");
      return;
    }
    pool1_out = malloc(CONV1_OUT_CH * POOL1_OUT_DIM * POOL1_OUT_DIM);
    if (pool1_out == NULL)
    {
      printf("pool1_out malloc failed...\n");
      return;
    }
    conv2_out = malloc(CONV2_OUT_CH * CONV2_OUT_DIM * CONV2_OUT_DIM);
    if (conv2_out == NULL)
    {
      printf("conv2_out malloc failed...\n");
      return;
    }
    pool2_out = malloc(CONV2_OUT_CH * POOL2_OUT_DIM * POOL2_OUT_DIM);
    if (pool2_out == NULL)
    {
      printf("pool2_out malloc failed...\n");
      return;
    }
    interface_out = malloc(INTERFACE_OUT_DIM);
    if (interface_out == NULL)
    {
      printf("interface_out malloc failed...\n");
      return;
    }
    linear_out = malloc(LINEAR_OUT_DIM);
    if (linear_out == NULL)
    {
      printf("linear_out malloc failed...\n");
      return;
    }
    y_out = malloc(LINEAR_OUT_DIM);
    if (y_out == NULL)
    {
      printf("y_out malloc failed...\n");
      return;
    }
    conv_buffer = malloc(MAX_CONV_BUFFER_SIZE);
    if (conv_buffer == NULL)
    {
      printf("conv_buffer malloc failed...\n");
      return;
    }
    fc_buffer = malloc(MAX_FC_BUFFER);
    if (fc_buffer == NULL)
    {
      printf("fc_buffer malloc failed...\n");
      return;
    }
  }
}

void run_nn_sn_classify(q7_t *input_data, q7_t *output_data)
{
  for (int i = 0; i < CONV1_IN_CH * CONV1_IN_DIM * CONV1_IN_DIM; i++)
  {
    input_data[i] = input_data[i] - 127;
  }
  arm_convolve_HWC_q7_basic(input_data, CONV1_IN_DIM, CONV1_IN_CH, conv1_w, CONV1_OUT_CH, CONV1_KER_DIM, CONV1_PAD, CONV1_STRIDE, conv1_b, CONV1_BIAS_LSHIFT, CONV1_OUT_RSHIFT, conv1_out, CONV1_OUT_DIM, (q15_t *)conv_buffer, fc_buffer);
  arm_maxpool_q7_HWC(conv1_out, POOL1_IN_DIM, POOL1_IN_CH, POOL1_KER_DIM, POOL1_PAD, POOL1_STRIDE, POOL1_OUT_DIM, NULL, pool1_out);
  arm_relu_q7(pool1_out, POOL1_OUT_DIM * POOL1_OUT_DIM * CONV1_OUT_CH);
  arm_convolve_HWC_q7_basic(pool1_out, CONV2_IN_DIM, CONV2_IN_CH, conv2_w, CONV2_OUT_CH, CONV2_KER_DIM, CONV2_PAD, CONV2_STRIDE, conv2_b, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_out, CONV2_OUT_DIM, (q15_t *)conv_buffer, NULL);
  arm_maxpool_q7_HWC(conv2_out, POOL2_IN_DIM, POOL2_IN_CH, POOL2_KER_DIM, POOL2_PAD, POOL2_STRIDE, POOL2_OUT_DIM, NULL, pool2_out);
  arm_relu_q7(pool2_out, POOL2_OUT_DIM * POOL2_OUT_DIM * CONV2_OUT_CH);
  // printf("1\n");
  // arm_convolve_HWC_q7_basic(pool2_out, CONV3_IN_DIM, CONV3_IN_CH, conv3_w, CONV3_OUT_CH, CONV3_KER_DIM,
	// 					  CONV3_PAD, CONV3_STRIDE, conv3_b, CONV3_BIAS_LSHIFT, CONV3_OUT_RSHIFT, conv3_out,
	// 					  CONV3_OUT_DIM, (q15_t *) conv_buffer, NULL);
  // arm_relu_q7(conv3_out, CONV3_OUT_DIM * CONV3_OUT_DIM * CONV3_OUT_CH);
  // printf("2\n");
  arm_fully_connected_q7_opt(pool2_out, interface_w, INTERFACE_IN_DIM, INTERFACE_OUT_DIM, INTERFACE_BIAS_LSHIFT, INTERFACE_OUT_RSHIFT, interface_b, interface_out, (q15_t *)fc_buffer);
  arm_relu_q7(interface_out, INTERFACE_OUT_DIM);
  arm_fully_connected_q7_opt(interface_out, linear_w, LINEAR_IN_DIM, LINEAR_OUT_DIM, LINEAR_BIAS_LSHIFT, LINEAR_OUT_RSHIFT, linear_b, output_data, (q15_t *)fc_buffer);
}
