# CMSIS-NN cifar10 example

The model of this example is from [[ARM-software](https://github.com/ARM-software)/**[ML-examples](https://github.com/ARM-software/ML-examples)**] and can be deployed on Arm Cortex-M CPUs using [CMSIS-NN](https://github.com/ARM-software/CMSIS_5).

## Requirements:
- CMSIS-NN in Framework/knowing/cmsis_5
- TJpgDec in Framework/knowing/image_processing
- Enough stack size (recommend 10240) for finsh thread which can be changed in "RT-Thread Components->Command shell->finsh shell" by menuconfig.

## To run this demo:
- Place the photo where you want
- Run demo by type the command
  ``` 
  cmsisnn_demo /path/to/photo