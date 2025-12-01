# Tensorflow Lite for MCU

## 仓库说明

该仓库是由 tensorflow 仓库生成：

```
# 生成源码项目
# 不指定 'hello_world' 则生成 hello_world/image_recognition/micro_speech/... 等多个项目
# 不指定 'make' 则生成 arduino、esp-idf、keil、make、mbed 等多个工程
# 生成结果在 tensorflow/tensorflow/lite/micro/tools/make/gen/osx_x86_64/prj/hello_world/make
# 以下指令生成 hello_world 的源码项目，其中算子使用常规算子
make -f tensorflow/lite/micro/tools/make/Makefile generate_hello_world_make_project

# 以下指令生成 针对 arm cortex-m 平台使用 cmsis 算子优化的源码项目，详见 tensorflow 仓库的 tensorflow/tensorflow/lite/micro/kernels/cmsis-nn/README.md
make -f tensorflow/lite/micro/tools/make/Makefile TAGS=cmsis-nn generate_hello_world_make_project
```

当前使用的 tensorflow 版本为：`e30e1d1aedb` （2021-01-07）。

我们在其他平台使用常规算子，在 cortex-m 平台使用 cmsis 优化算子，需要将上述生成的两个源码项目整合起来，具体代码如何融合，见本文最后文件列表。

## 问题说明

### 问题一：缺少头文件路径

tensorflow生成的源码项目中的 makefile 的头文件路径是不全的，需要添加以下头文件路径：

```
/tensorflow/lite/micro/tools/make/downloads/cmsis
/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/DSP/Include
/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Include
/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/Core/Include
```

### 问题二：cmsis 缺少头文件

tensorflow生成的项目缺少 cmsis_gcc.h 头文件，我们将该文件放在了 patch/cmsis_gcc.h。

## 文件列表

```
# 共有文件
tensorflow/lite/micro/all_ops_resolver.cc
tensorflow/lite/micro/debug_log.cc
tensorflow/lite/micro/memory_helpers.cc
tensorflow/lite/micro/micro_allocator.cc
tensorflow/lite/micro/micro_error_reporter.cc
tensorflow/lite/micro/micro_interpreter.cc
tensorflow/lite/micro/micro_profiler.cc
tensorflow/lite/micro/micro_string.cc
tensorflow/lite/micro/micro_time.cc
tensorflow/lite/micro/micro_utils.cc
tensorflow/lite/micro/recording_micro_allocator.cc
tensorflow/lite/micro/recording_simple_memory_allocator.cc
tensorflow/lite/micro/simple_memory_allocator.cc
tensorflow/lite/micro/test_helpers.cc
tensorflow/lite/micro/benchmarks/keyword_scrambled_model_data.cc
tensorflow/lite/micro/memory_planner/greedy_memory_planner.cc
tensorflow/lite/micro/memory_planner/linear_memory_planner.cc
tensorflow/lite/micro/testing/test_conv_model.cc
tensorflow/lite/c/common.c
tensorflow/lite/core/api/error_reporter.cc
tensorflow/lite/core/api/flatbuffer_conversions.cc
tensorflow/lite/core/api/op_resolver.cc
tensorflow/lite/core/api/tensor_utils.cc
tensorflow/lite/kernels/internal/quantization_util.cc
tensorflow/lite/kernels/kernel_util.cc
tensorflow/lite/schema/schema_utils.cc
tensorflow/lite/micro/kernels/activations.cc
tensorflow/lite/micro/kernels/arg_min_max.cc
tensorflow/lite/micro/kernels/ceil.cc
tensorflow/lite/micro/kernels/circular_buffer.cc
tensorflow/lite/micro/kernels/comparisons.cc
tensorflow/lite/micro/kernels/concatenation.cc
tensorflow/lite/micro/kernels/conv_test_common.cc
tensorflow/lite/micro/kernels/dequantize.cc
tensorflow/lite/micro/kernels/detection_postprocess.cc
tensorflow/lite/micro/kernels/elementwise.cc
tensorflow/lite/micro/kernels/ethosu.cc
tensorflow/lite/micro/kernels/flexbuffers_generated_data.cc
tensorflow/lite/micro/kernels/floor.cc
tensorflow/lite/micro/kernels/hard_swish.cc
tensorflow/lite/micro/kernels/kernel_runner.cc
tensorflow/lite/micro/kernels/kernel_util.cc
tensorflow/lite/micro/kernels/l2norm.cc
tensorflow/lite/micro/kernels/logical.cc
tensorflow/lite/micro/kernels/logistic.cc
tensorflow/lite/micro/kernels/maximum_minimum.cc
tensorflow/lite/micro/kernels/neg.cc
tensorflow/lite/micro/kernels/pack.cc
tensorflow/lite/micro/kernels/pad.cc
tensorflow/lite/micro/kernels/prelu.cc
tensorflow/lite/micro/kernels/quantize.cc
tensorflow/lite/micro/kernels/quantize_common.cc
tensorflow/lite/micro/kernels/reduce.cc
tensorflow/lite/micro/kernels/reshape.cc
tensorflow/lite/micro/kernels/resize_nearest_neighbor.cc
tensorflow/lite/micro/kernels/round.cc
tensorflow/lite/micro/kernels/shape.cc
tensorflow/lite/micro/kernels/split.cc
tensorflow/lite/micro/kernels/split_v.cc
tensorflow/lite/micro/kernels/strided_slice.cc
tensorflow/lite/micro/kernels/sub.cc
tensorflow/lite/micro/kernels/svdf_common.cc
tensorflow/lite/micro/kernels/tanh.cc
tensorflow/lite/micro/kernels/transpose_conv.cc
tensorflow/lite/micro/kernels/unpack.cc
tensorflow/lite/micro/examples/hello_world/main.cc
tensorflow/lite/micro/examples/hello_world/main_functions.cc
tensorflow/lite/micro/examples/hello_world/model.cc
tensorflow/lite/micro/examples/hello_world/output_handler.cc
tensorflow/lite/micro/examples/hello_world/constants.cc

# 常规算子，将这些文件移入 tensorflow/lite/micro/kernels/normal 中
tensorflow/lite/micro/kernels/add.cc
tensorflow/lite/micro/kernels/conv.cc
tensorflow/lite/micro/kernels/depthwise_conv.cc
tensorflow/lite/micro/kernels/fully_connected.cc
tensorflow/lite/micro/kernels/mul.cc
tensorflow/lite/micro/kernels/pooling.cc
tensorflow/lite/micro/kernels/softmax.cc
tensorflow/lite/micro/kernels/svdf.cc

# cmsis 优化算子
tensorflow/lite/micro/kernels/cmsis-nn/add.cc
tensorflow/lite/micro/kernels/cmsis-nn/conv.cc
tensorflow/lite/micro/kernels/cmsis-nn/depthwise_conv.cc
tensorflow/lite/micro/kernels/cmsis-nn/fully_connected.cc
tensorflow/lite/micro/kernels/cmsis-nn/mul.cc
tensorflow/lite/micro/kernels/cmsis-nn/pooling.cc
tensorflow/lite/micro/kernels/cmsis-nn/softmax.cc
tensorflow/lite/micro/kernels/cmsis-nn/svdf.cc
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_relu6_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_add_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_mul_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_w.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_x.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_y.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_z.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1_x_n_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_HWC_q7_fast_nonsquare.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_s8_fast.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_basic.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast_nonsquare.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_RGB.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic_nonsquare.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast_nonsquare.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_wrapper_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_3x3_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8_opt.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_u8_basic_ver1.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_wrapper_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7_nonsquare.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_depthwise_conv_s8_core.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15_reordered.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16_reordered.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15_opt.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15_opt.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7_opt.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_accumulate_q7_to_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_add_q7.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_padded_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_1x_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_4x_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mult_nt_t_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q7.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nntables.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_no_shift.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_no_shift.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_with_offset.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_with_offset.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/PoolingFunctions/arm_avgpool_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/PoolingFunctions/arm_pool_q7_HWC.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ReshapeFunctions/arm_reshape_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SVDFunctions/arm_svdf_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q15.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q7.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_u8.c
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_with_batch_q7.c
```
