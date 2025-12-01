# CMSIS-NN vegetable classify example

This example uses CMSIS-NN to classify vegetable in real time under certain circumstances .

## Requirements:
- CMSIS-NN in Framework/knowing/cmsis_5
- **ov2640 need to be configured** in menuconfig "More Drivers->ov2640 driver" as follows
  - Output format (RGB565 mode)
  - (256) X direction resolution of outputimage
  - (256) Y direction resolution of outputimage
  - (512) X direction WINDOWS SIZE
  - (512) Y direction WINDOWS SIZE

## To run this demo:
- Set up and configure the corresponding hardware environment.

- Run demo by type the command
  ``` 
  cmsisnn_vegetable_classify

## Results

- **tomato**

![tomato](https://www.gitlink.org.cn/repo/WentaoWong/xiuos/raw/branch/dev/APP_Framework/Applications/knowing_app/cmsis_5_demo/cmsisnn_vegetable_classify/doc/tomato.jpg)

- **potato**

![potato](https://www.gitlink.org.cn/repo/WentaoWong/xiuos/raw/branch/dev/APP_Framework/Applications/knowing_app/cmsis_5_demo/cmsisnn_vegetable_classify/doc/potato.jpg)

- **pepper**

![pepper](https://www.gitlink.org.cn/repo/WentaoWong/xiuos/raw/branch/dev/APP_Framework/Applications/knowing_app/cmsis_5_demo/cmsisnn_vegetable_classify/doc/pepper.jpg)

- **mushroom**

![mushroom](https://www.gitlink.org.cn/repo/WentaoWong/xiuos/raw/branch/dev/APP_Framework/Applications/knowing_app/cmsis_5_demo/cmsisnn_vegetable_classify/doc/mushroom.jpg)