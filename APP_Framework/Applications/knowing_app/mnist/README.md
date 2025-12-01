# MNIST 说明

要使用本例程，MCU RAM必须至少500K左右，所以本例程目前在K210上面验证过，stm32f407 目前在rtt上原则上只能采取dlmodule加载的方式。

![K210 mnist ](E:\XIUOS_FRAMEWORK\xiuos\APP_Framework\Applications\knowing_app\mnist\K210 mnist .png)

## 使用

tools/mnist-train.py 训练生成 mnist 模型。

tools/mnist-inference.py 使用 mnist 模型进行推理。

tools/mnist-c-model.py 将 mnist 模型转换成 C 的数组保存在 model.h 中。

tools/mnist-c-digit.py 将 mnist 数据集中的某个数字转成数组保存在 digit.h 中。

## 参考资料

https://tensorflow.google.cn/lite/performance/post_training_quantization

https://tensorflow.google.cn/lite/performance/post_training_integer_quant

https://colab.research.google.com/github/tensorflow/tensorflow/blob/master/tensorflow/lite/micro/examples/hello_world/train/train_hello_world_model.ipynb