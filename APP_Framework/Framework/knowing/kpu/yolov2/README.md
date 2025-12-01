# KPU(K210) YOLOv2 region layer

## Introduction

KPU(k210) accelerate most of CNN network layers, but do not support some of operators of YOLOv2 region layer. Such layers and operators will run on MCU instead.  
YOLOv2 region layer accept feature map(shape w\*h\*c) and return final detection boxes.

## Usage

Use `(scons --)menuconfig` in bsp folder *(Ubiquitous/RT_Thread/bsp/k210)*, open *APP_Framework --> Framework --> support knowing framework --> kpu model postprocessing --> yolov2 region layer*.