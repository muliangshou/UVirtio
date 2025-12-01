# OTA README

xiuos当前的ota功能允许应用bin文件可以通过4G实现远程的bin文件更新（限制：1、bin文件存放在设备SD卡并且应用从SD卡启动；2、暂且支持4G实现；3、暂时只支持aiit终端;4、只支持xiuos内核）。

## 文件说明

| 名称 | 说明 |
| -- | -- |
| ota.c| xiuos设备OTA代码 |
| ota_server.c | pc服务端的实例代码供参考 |


## 使用说明
xiuos的应用bin文件更新依赖上层的adapter框架，因此需要在menuconfig同时配置以下选项：
1、ota开关APPLICATION_OTA打开；
2、adapter的4G功能开关；
3、拆分的应用启动SEPARATE_COMPILE开关从SD卡启动的配置开关APP_STARTUP_FROM_SDCARD。


