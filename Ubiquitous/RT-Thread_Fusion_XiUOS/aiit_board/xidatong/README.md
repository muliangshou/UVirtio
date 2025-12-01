# XiDaTong_ARM_Reference_Resource

## 1. 矽达通介绍

矽达通外观图:

![](https://s2.loli.net/2022/03/17/56PYhSplI1Ud89a.png)

拆开后盖:

![](https://s2.loli.net/2022/03/17/5SkJish7VqRfwrX.png)

矽达通烧录这里介绍两种方式，分别是 NXP-MCUBootUtility 和 Keil MDK5

## 2. NXP-MCUBootUtility 方式烧录

**[NXP-MCUBootUtility](https://github.com/JayHeng/NXP-MCUBootUtility/tree/v3.4.0)** 是一款开源免费的专为 NXP MCU 安全启动而设计的 GUI 工具。目前主要支持 i.MXRT、LPC、Kinetis 系列 MCU 芯片

> ### 测试环境

- Windows

- NXP-MCUBootUtility v3.4.0

> ### 烧录流程

将矽达通串口1通过 usb 转串口连接至电脑，并在电脑端查看到相应端口，如下图，com15

![](https://s2.loli.net/2022/03/17/b2alJPrmvcEXh5Z.png)![](https://s2.loli.net/2022/03/17/lu2jfeGnyzHKZpD.png)

打开 NXP-MCUBootUtility.exe

确保一下配置选项正确，COM Port记得选择上述对应的串口

![](https://s2.loli.net/2022/03/16/3z2NLbqlS4f6VGY.png)

将拨码开关拨到 1 on 2 on 3 off 4 off 进入 Serial Download Programming(SDP) 模式, 重新连接电源

先点击 Connect to ROM，若连接成功按钮会变蓝

![](https://s2.loli.net/2022/03/16/9pefoEvilVYSOrh.png)

在 Application Image File 一栏中选择要烧录的 elf 文件，文件格式选择 .out(elf) from GCC ARM，然后点击 All-In-One Action 烧录即可

![](https://s2.loli.net/2022/03/16/SrEywuekORU2sNz.png)

烧录完后后，将拨码开关拨回 1 off 2 off 3 off 4 off 进入 nor-flash 启动模式，重新上电，即可从 QSPI Flash 启动程序，此时可以从串口看到调试信息

`注：由于采用串口烧录，在连接和烧录的时候记得先关闭串口调试工具`

## 2. Keil MDK 方式烧录

Keil MDK 是一系列基于 Arm Cortex-M 的微控制器设备的完整软件开发环境，可以自行去官网 **[购买下载](https://www.keil.com/download/product/)** ，需要注意的是请选择使用 Keil MDK5.24a 及以上版本

开发板连接 CMSIS-DAP 调试器，如下图所示，连接 DIO/CLK/GND 对应的三个引脚即可

![](https://s2.loli.net/2022/03/17/V6sQ8R5SXHMbZdU.png)

> ### 芯片支持包导入

去keil官网 [http://www.keil.com/dd2/pack/#/eula-container](http://www.keil.com/dd2/pack/#/eula-container) 下载对应的板级安装包

![](https://s2.loli.net/2022/03/17/Fofnw1ajkuyOLmi.png)

注意： 安装包可能随时间推移，版本有所改动，可选择最新版本下载。

下载完成，安装即可。

> ### 烧写固件安装

将开发板烧写固件复制到 Keil5 安装目录 Keil_v5\ARM\Flash\MIMXRT_QSPIFLASH.FLM，该固件可以兼容32M以下nor Flash烧写。

例如： c:\Keil_v5\ARM\Flash\MIMXRT_QSPIFLASH.FLM

> ### Keil 环境配置

查看仿真器是否连接成功：

![](https://s2.loli.net/2022/03/17/xmQqBslFL1j2SDN.png)

![](https://s2.loli.net/2022/03/17/H5it68ZWzuRwA4J.png)

> ### 添加烧写固件

选择 Flash Download，删除原来的烧写配置

![](https://s2.loli.net/2022/03/17/xOZCnXdaErIF5oS.png)

添加该烧写固件，size 大小为 32MB

![](https://s2.loli.net/2022/03/17/xfCIsgOF2N5t3pM.png)

下载的一些设置记得勾上：

![](https://s2.loli.net/2022/03/17/acizCh5tH7MJATO.png)![](https://s2.loli.net/2022/03/17/cUqhyRgoPFQHW6L.png)

然后编译烧录即可，keil下载不需要进入 SDP 模式，无需拨拨码开关。



## 3.矽达通硬件资源

|      端口      |        功能        |
| :------------: | :----------------: |
|     uart1      |       shell        |
|     uart3      | 485CH1（外围接口） |
|     uart4      | 485CH2（外围接口） |
|     uart8      |   ec200t 4G 通讯   |
|     uart2      |     wifi esp07     |
|  ch438  EXTU2  |   Bluetooth HC08   |
|  ch438  EXTU3  | Lora  E220-400T22S |
|  ch438  EXTU1  | zigbee  E18-MS1PA1 |
|       SD       |        sd卡        |
|      usb1      |   ec200t 4G通讯    |
|      usb2      |    外围usb接口     |
|      can       |    can外围接口     |
|      IIC       |        屏幕        |
| 其他CH438 EXTU |      外围接口      |



