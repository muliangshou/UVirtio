# 写在前面

如果期望该框架运行在Nuttx系统上，请阅读此文档，否则请跳过即可。

## 1、下载nuttx及apps代码

nuttx代码及apps代码以子仓的形式托管在xiuos\Ubiquitous\Nuttx下，apps和nuttx目录为空的，进行编译前需要下载该代码

如果您的代码是用git的形式clone下来的，那么clone后，还需要执行如下操作：

```shell
cd xiuos
git submodule
git submodule init
git submodule update Ubiquitous/Nuttx_Fusion_XiUOS/apps
git submodule update Ubiquitous/Nuttx_Fusion_XiUOS/nuttx
```

执行完成后在Ubiquitous/Nuttx目录下，apps和nuttx的代码都被下载下来，当前的版本是nuttx-10.2.0版本。

## 2、感 - 传感器框架支持Nuttx

目前感框架下面的传感器有I2C和串口两种，使用两种方式的传感器在APP_Framework下的修改大同小异，这里以hs300x传感器测量温度为例，说明增加一款传感器适配Nuttx时APP_Framework目录下需要修改的文件：

### APP_Framework目录下修改

```shell
└── APP_Framework
    ├── Applications
    │   ├── framework_init.c
    │   ├── Make.defs
    │   ├── Makefile
    │   └── sensor_app
    │       ├── Kconfig
    │       ├── Make.defs
    │       ├── Makefile
    │       └── temperature_hs300x.c
    ├── Framework
    │   ├── Make.defs
    │   └── sensor
    │       ├── Make.defs
    │       ├── Makefile
    │       ├── sensor.c
    │       └── temperature
    │           ├── hs300x_temp
    │           │   ├── hs300x_temp.c
    │           │   ├── Make.defs
    │           │   └── Makefile
    │           ├── Kconfig
    │           ├── Make.defs
    │           └── Makefile
    ├── Make.defs
    └── Makefile
```

(1)在上述文件中，Make.defs文件是Nuttx特有的文件描述，每个Make.defs有两个作用：

​	①根据对应的宏控制是否为CONFIGURED_APPS添加需要编译的目录

​	②包含子目录中的所有Make.defs，向上传递

(2)Makefile文件是多个操作系统共用的，里面用CONFIG_ADD_XXXX_FETURES宏来区别不同的操作系统

(3)Kconfig文件也是多个操作系统共用的，不同操作系统配置时略有差异也用CONFIG_ADD_XXXX_FETURES进行了控制。

(4)*.c文件的针对适配Nuttx的修改一般是头文件的包含或者接口的调用需要区分不同的操作系统

### Ubiquitous目录下修改

#### 使用I2C外设--以hs300x为例

```shell
└── Nuttx
    └── app_match_nuttx
        ├── apps
        │   ├── Makefile
        │   └── nshlib
        │       ├── Kconfig
        │       ├── Makefile
        │       ├── nsh_Applicationscmd.c
        │       ├── nsh_command.c
        │       └── nsh.h
        ├── build.sh
        └── nuttx
            ├── boards
            │   └── arm
            │       └── stm32
            │           ├── common
            │           │   ├── include
            │           │   │   └── stm32_hs300x.h
            │           │   └── src
            │           │       ├── Make.defs
            │           │       └── stm32_hs300x.c
            │           └── stm32f4discovery
            │               ├── scripts
            │               │   └── Make.defs
            │               └── src
            │                   └── stm32_bringup.c
            ├── drivers
            │   └── sensors
            │       ├── hs300x.c
            │       ├── Kconfig
            │       └── Make.defs
            ├── include
            │   └── nuttx
            │       └── sensors
            │           └── hs300x.h
            ├── Kconfig
            ├── Makefile
            └── tools
                ├── cfgdefine.c
                └── Makefile.unix
```

适配Nuttx系统需要修改Nuttx原生代码中多处文件，为了不破坏Nuttx原生代码，我们可以先在Nuttx原生代码中进行修改，然后将修改的文件导出到app_match_nuttx目录下，该目录下的文件结构与Nuttx原生代码完全一致，在编译前在app_match_nuttx目录执行一下source build.sh即可将修改同步到nuttx中，进行编译。

(1)app_match_nuttx/apps下的Makefile文件进行了一处修改，把APP_Framework目录最上层的Make.defs包含进来，这样在编译的时候就会引导进入APP_Framework目录进行相应的编译，该处已经修改完成，后续不用再去修改

```makefile
include $(APPDIR)/../../../APP_Framework/Make.defs
```

(2)app_match_nuttx/apps/nshlib下的修改，Nuttx下应用的编译方式有两种，一种是编译在Builtin Apps下，另一种是编译成系统命令cmd，为了与XiUos和rt-thread等其他操作系统保持一致，采用后面一种编译方式。具体修改参见该目录下的修改即可，这里不加赘述。

(3)app_match_nuttx/nuttx下修改：

目前感框架的传感器使用的外设有I2C和串口两种，Nuttx针对串口外设的驱动做了比较完善支持，因此使用串口的传感器只需要去使能相应的串口即可使用，目前ps5308传感器就是这种方式，使用I2C和SPI外设的传感器，目前Nuttx的做法是在nuttx/drivers/sensors进行传感器的注册，主要是定义open/close/read/write等标准的操作接口。传感器的注册这一过程主要涉及到文件主要是上图中的

stm32_hs300x.h

stm32_hs300x.c

stm32f4discovery/stm32_bringup.c(注意支持不同的板子需要在对应的文件下去修改，这里当前是stm32f4discovery)

hs300x.c

hs300x.h

(4)nuttx/Makefile的修改，在这个文件里只有一出修改，设置了一个环境变量 KERNEL_ROOT，这个环境变量在APP_Framework中会用到。该处已经修改完成，后续不用再去修改

```makefile
export KERNEL_ROOT = $(CURDIR)
```

(5) nuttx/tools/Makefile.unix的修改，这个文件中新定义了一个环境变量APPPATHS，将APP_Framework中所以的头文件的路径包含进去，如果后面有新的头文件加入，一定需要在这里加入新头文件的路径；设置了一个环境变量SRC_APP_DIR这个环境变量在APP_Framework中会用到。

```makefile
export APPPATHS = -I$(APPDIR)/../../../APP_Framework/Framework/sensor 
APPPATHS += -I$(APPDIR)/../../../APP_Framework/Applications/general_functions/list
APPPATHS += -I$(APPDIR)/../../../APP_Framework/Framework/transform_layer/nuttx

export SRC_APP_DIR = ../../../APP_Framework
```

(6)要使(5)中的修改生效，需要修改对应板子的Make.defs文件，以stm32f4discovery为例，在nuttx/boards/arm/stm32/stm32f4discovery/scripts/Make.defs中CFLAGS编译选项加上在(5)中设置的APPPATHS

```makefile
CFLAGS := $(APPPATHS) $(ARCHCFLAGS) $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(ARCHDEFINES) $(EXTRAFLAGS) -pipe
```

该Make.defs文件在实际编译时会被拷贝至nuttx目录下

#### 使用串口外设--以ps5308传感器为例

```
.
├── APP_Framework
│   ├── Applications
│   │   ├── framework_init.c
│   │   └── sensor_app
│   │       ├── Kconfig
│   │       ├── Makefile
│   │       ├── pm10_0_ps5308.c
│   │       ├── pm1_0_ps5308.c
│   │       └── pm2_5_ps5308.c
│   └── Framework
│       └── sensor
│           └── pm
│               ├── Kconfig
│               ├── Make.defs
│               └── ps5308
│                   ├── Make.defs
│                   ├── Makefile
│                   └── ps5308.c
└── Ubiquitous
    └── Nuttx
        └── app_match_nuttx
            ├── apps
            │   └── nshlib
            │       ├── Kconfig
            │       ├── nsh_Applicationscmd.c
            │       ├── nsh_command.c
            │       └── nsh.h
            └── nuttx
                └── arch
                    └── arm
                        └── src
                            └── stm32
                                └── stm32_serial.c
```

(1)Nuttx_Fusion_XiUOS/app_match_nuttx/apps/nshlib下修改，将应用编译成cmd形式，与上面HS300x处描述一致，参考上面即可

(2)/nuttx/arch/arm/src/stm32/stm32_serial.c，此处代码原本是会将所有打开的串口从0开始依次注册为ttyS1-ttySN，比较不方便，例如打开了USART1、USART3、USART5，USART3将会被注册为ttyS2，容易引起歧义，这边做的修改会将USARTN，注册为ttySN。

其他架构的板卡，如K210，需要在nuttx/arch/risc-v/src/k210/k210_serial.c进行适配修改。

## 3、硬件支持

目前Nuttx支持ARM和RISC-V两种架构的微处理器:

### ARM

ARM架构系列的开发板有

	stm32f407-st-discovery

### RISC-V

RISC-V架构系列的开发板有

​	

## 4、开发环境

### 推荐使用：

### 操作系统： [Ubuntu18.04](https://ubuntu.com/download/desktop)

### 开发工具： [VSCode](http://101.36.126.201:8011/vscode_1.55.2-1618307277_amd64.deb)

### 依赖包安装：

```
$ sudo apt install build-essential pkg-config  git
$ sudo apt install gcc make libncurses5-dev openssl libssl-dev bison flex libelf-dev autoconf libtool gperf libc6-dev
```

### 裁减配置工具：

kconfig-frontends工具地址： https://forgeplus.trustie.net/projects/xuos/kconfig-frontends，下载与安装的具体命令如下：

```shell
mkdir kfrontends  && cd kfrontends
git  clone https://git.trustie.net/xuos/kconfig-frontends.git
```

下载源码后按以下步骤执行软件安装：

```shell
cd kconfig-frontends
 ./xs_build.sh
```

### 编译工具链：

ARM： arm-none-eabi，默认安装到Ubuntu的/usr/bin/arm-none-eabi-，使用如下命令行下载

```shell
mkdir kfrontends  && cd kfrontends
git  clone https://git.trustie.net/xuos/kconfig-frontends.git
```

## 5、编译及配置

#### 在Nuttx\app_match_nuttx目录下执行

```shell
source build.sh
```

#### 执行完毕会跳转到Nuttx\nuttx目录，执行

```shell
./tools/configure.sh stm32f4discovery:nsh  (应用内核一起编译)

./tools/configure.sh stm32f4discovery:kostest  (应用内核分开编译)
视情况而定，如果需要前面加sudo
```

#### 然后执行

```shell
make  menuconfig
```

##### 开启Nuttx Support CLOCK_MONOTONIC

进入RTOS Features > Clocks and Timers，勾选上Support CLOCK_MONOTONIC

##### 使用I2C外设--以hs300x温度传感器为例说明配置过程：

①进入XIUOS features > APP_Framework > Framework ，选择操作系统(这一步一定要最先配置)

②依次进入XIUOS features > APP_Framework > Framework > support sensor framework > Using temperature sensor device 

​	进行sensor name、quantity name、device name(这里是一个字符设备的路径)、i2c address的配置。注意这里设置的 HS300x device name 设置的/dev/i2cN需要与第④步勾选的i2cN保持一致

③依次进入XIUOS features > APP_Framework > Applications > sensor app > Using sensor apps > Using sensor temperature apps ，并进行勾选

④依次进入System Type > STM32 Peripheral Support ，勾选传感器使用的I2C外设。

⑤ 进入Device Drivers > I2C Driver Support ，找到I2C Driver Support ，勾选上I2C Slave

⑥在Device Drivers 下找到Sensor Device Support勾选上后进入，勾选HS300X Temperature and Humidity Sensor support并设置频率

退出menuconfig界面，注意记得保存，此时会在nuttx目录下生成一个.config文件。

##### 使用串口外设--以ps5308传感器pm1.0为例说明配置过程：

①进入XIUOS features > APP_Framework > Framework ，选择操作系统(这一步一定要最先配置)

②依次进入XIUOS features > APP_Framework > Framework > support sensor framework > Using PM sensor device，进行相应的勾选与设置，注意这里设置的 PS5308 device name 设置的/dev/ttySN需要与第④步勾选的USART-N保持一致

③依次进入XIUOS features > APP_Framework > Applications > sensor app > Using sensor apps > Using sensor PM1.0 apps，并进行勾选

④依次进入System Type > STM32 Peripheral Support ，勾选传感器使用的USART外设。

⑤ 依次进入RTOS Features > Tasks and Scheduling，勾选Support parent/child task relationships及Retain child exit status

⑥ 依次进入Device Drivers > Serial Driver Support > USARTN Configuration进行外设buffesize大小、波特率等设置。

退出menuconfig界面，注意记得保存，此时会在nuttx目录下生成一个.config文件。



#### 在当前目录执行编译

```shell
make
或
make -j8
```

make时加上V=1参数可以看到较为详细的编译信息，但是编译过程会比较慢。最后在nuttx下会编译出一个nuttx.bin文件(应用内核一起编译)

