# Face detection demo

### A face object detection task demo. Running MobileNet-yolo on K210-based edge devices.

---

## Training

kmodel from [GitHub](https://github.com/kendryte/kendryte-standalone-demo/blob/develop/face_detect/detect.kmodel).

## Deployment

### compile and burn

Use `(scons --)menuconfig` in bsp folder *(Ubiquitous/RT_Thread/bsp/k210)*, open:

- More Drivers --> ov2640 driver
- Board Drivers Config --> Enable LCD on SPI0
- Board Drivers Config --> Enable SDCARD (spi1(ss0))
- Board Drivers Config --> Enable DVP(camera)
- RT-Thread Components --> POSIX layer and C standard library --> Enable pthreads APIs
- APP_Framework --> Framework --> support knowing framework --> kpu model postprocessing --> yolov2 region layer
- APP_Framework --> Applications --> knowing app --> enable apps/face detect

`scons -j(n)` to compile and burn in by *kflash*.

### json config and kmodel

Copy json config for deployment o SD card */kmodel*. Example config file is *detect.json* in this directory. Copy final kmodel to SD card */kmodel* either.

---

## Run

In serial terminal, `face_detect` to start a detection thread, `face_detect_delete` to stop it. Detection results can be found in output.
