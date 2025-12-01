# Neural Network on Microcontroller (NNoM)

NNoM is a high-level inference Neural Network library specifically for microcontrollers, released under Apache License 2.0. 

Current version is 0.4.3. More information available in [NNOM](https://github.com/majianjia/nnom).

## CMSIS-NN Backend

[CMSIS-NN/DSP](https://github.com/ARM-software/CMSIS_5/tree/develop/CMSIS/NN) is an inference acceleration libraries for Arm Cortex-M CPUs and can be used as the backend of NNoM for high performance.

## Notes

- CHW format is incompatible with CMSIS-NN and must be used when using hardware accelerator such as KPU in K210 chip.
- Static memory buffer must be set by using "nnom_set_static_buf()" before creating a model.