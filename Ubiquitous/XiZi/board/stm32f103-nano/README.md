# 从零开始构建矽璓工业物联操作系统：使用ARM架构的STM32F103-nano开发板

[XiUOS](http://xuos.io/) (X Industrial Ubiquitous Operating System) 矽璓工业物联操作系统是一款面向工业物联场景的泛在操作系统，来自泛在操作系统研究计划。所谓泛在操作系统(UOS: Ubiquitous Operating Systems)，是支持互联网时代人机物融合泛在计算应用模式的新型操作系统，是传统操作系统概念的泛化与延伸。在泛在操作系统技术体系中，不同的泛在计算设备和泛在应用场景需要符合各自特性的不同UOS，XiUOS即是面向工业物联场景的一种UOS，主要由一个极简的微型实时操作系统(RTOS)内核和其上的智能工业物联框架构成，支持工业物联网(IIoT: Industrial Internet of Things)应用。

## 开发环境搭建

### 推荐使用：

**操作系统：** ubuntu18.04 [https://ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)

更新`ubuntu 18.04`源的方法:（根据自身情况而定，可以不更改）

第一步:打开sources.list文件

```c
sudo vim /etc/apt/sources.list
```

第二步:将以下内容复制到sources.list文件

```c
deb http://mirrors.aliyun.com/ubuntu/ bionic main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-proposed main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-backports main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-proposed main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-backports main restricted universe multiverse
```

第三步:更新源和系统软件

```c
sudo apt-get update
sudo apt-get upgrade
```

**开发工具推荐使用 VSCode   ，VScode下载地址为：** VSCode  [https://code.visualstudio.com/](https://code.visualstudio.com/)，推荐下载地址为 [http://vscode.cdn.azure.cn/stable/3c4e3df9e89829dce27b7b5c24508306b151f30d/code_1.55.2-1618307277_amd64.deb](http://vscode.cdn.azure.cn/stable/3c4e3df9e89829dce27b7b5c24508306b151f30d/code_1.55.2-1618307277_amd64.deb)

### 依赖包安装：

```
$ sudo apt install build-essential pkg-config  git
$ sudo apt install gcc make libncurses5-dev openssl libssl-dev bison flex libelf-dev autoconf libtool gperf libc6-dev
```

**XiUOS操作系统源码下载：** XiUOS [https://www.gitlink.org.cn/xuos/xiuos](https://www.gitlink.org.cn/xuos/xiuos)

新建一个空文件夹并进入文件夹中，并下载源码，具体命令如下：

```c
mkdir test  &&  cd test
git clone https://gitlink.org.cn/xuos/xiuos.git
```

1、打开XiUOS源码文件包可以看到以下目录：
| 名称 | 说明 |
| -- | -- |
| APP_Framework | 应用代码 |
| Ubiquitous | 板级支持包,支持NuttX、RT-Thread和XiZi内核 |

2、打开XiZi内核源码文件包可以看到以下目录：
| 名称 | 说明 |
| -- | -- |
| arch | 架构代码 |
| board | 板级支持包 |
| fs | 文件系统 |
| kernel | 内核源码 |
| lib | 第三方库源码 |
| resources | 驱动文件 |
| tool | 系统工具 |

使用VScode打开代码，具体操作步骤为：在源码文件夹下打开系统终端，输入`code .`即可打开VScode开发环境，如下图所示：

<div align= "center"> 
<img src = img/vscode.jpg  width =1000>
  </div>

### 裁减配置工具的下载

裁减配置工具：

**工具地址：** kconfig-frontends [https://www.gitlink.org.cn/xuos/kconfig-frontends](https://www.gitlink.org.cn/xuos/kconfig-frontends)，下载与安装的具体命令如下：

```c
mkdir kfrontends  && cd kfrontends
git clone https://gitlink.org.cn/xuos/kconfig-frontends.git
```

下载源码后按以下步骤执行软件安装：

```c
cd kconfig-frontends
./xs_build.sh
```

### 编译工具链：

ARM： arm-none-eabi(`gcc version 6.3.1`)，默认安装到Ubuntu的/usr/bin/arm-none-eabi-，使用如下命令行下载和安装。

```shell
$ sudo apt install gcc-arm-none-eabi
```

# 在STM32F103-nano上创建第一个应用 --helloworld

## 1. 简介

| 硬件 | 描述 |
| -- | -- |
|芯片型号| Stm32F103RBT6|
|CPU|arm cortex-m3|
|主频| 72MHz |
|片内SRAM| 20KB |
|片上FLASH| 128KB |
| 外设 | -- |
| | ADC、DAC、USB、GPIO、UART、SPI、SDIO、RTC、CAN、DMA、MAC、I²C、WDT、Timer等 |

XiUOS板级驱动当前支持使用GPIO、UART。

## 2. 编译说明

### 编辑环境：`Ubuntu18.04`

### 编译工具链：`arm-none-eabi-gcc`
使用`VScode`打开工程的方法有多种，本文介绍一种快捷键，在项目目录下将`code .`输入linux系统命令终端即可打开目标项目

修改`APP_Framework/Applications`文件夹下`main.c`
在输出函数中写入 `Hello, world!`完成代码编辑。

![main](img/main.png)

编译步骤：

1.在VScode命令终端中执行以下命令，生成配置文件。

```c
cd ./Ubiquitous/XiZi
make BOARD=stm32f103-nano distclean
make BOARD=stm32f103-nano menuconfig
```

2.在menuconfig界面配置需要关闭和开启的功能，按回车键进入下级菜单，按Y键选中需要开启的功能，按N键选中需要关闭的功能，配置结束后保存并退出（本例旨在演示简单的输出例程，所以没有需要配置的选项，双击快捷键ESC退出配置）。

![menuconfig1](img/menuconfig1.png)

退出时选择`yes`保存上面所配置的内容，如下图所示：

![menuconfig2](img/menuconfig2.png)

3.需要注意的是，stm32f103-nano开发板中CPU RAM大小为20KB，需要调整XiUOS中堆栈相关配置的默认值，避免出现超出堆栈大小而无法正常启动。

（1）ID hash table配置，ID num配置为16，减少.bss段大小;

![ID_HASH_TABLE](img/ID_HASH_TABLE.png)

（2）IPC feature配置，保留信号量、互斥锁等必需的进程间通信，关闭队列等未用到的方式;

![IPC_FEATURE](img/IPC_FEATURE.png)

（3）ENV task配置，该线程栈大小配置为1024;

![KTASK_SIZE](img/KTASK_SIZE.png)

（4）MAIN task配置，该线程栈大小配置为256;

![MAIN_STACK_SIZE](img/MAIN_STACK_SIZE.png)

（5）大内存分配页对齐配置，页对齐配置为1KB，减少页对齐后优化的ram空间，增加可分配的堆大小;

![MEMORY_PAGE_SIZE](img/MEMORY_PAGE_SIZE.png)

（6）SHELL task配置，该线程栈大小配置为4096，建议无优化空间时再选择减少SHELL线程栈大小，避免出现SHELL工作异常的情况;

![SHELL_STACK_SIZE](img/SHELL_STACK_SIZE.png)

（7）WORKQUEUE配置，若未用到工作队列，可关闭该功能，减少堆栈空间申请;

![WORKQUEUE](img/WORKQUEUE.png)

（7）ZOMBIE RECYCLE task配置 ，僵尸线程回收线程栈大小同样在无优化空间时可选择减少。

![ZOMBIE_STACK_SIZE](img/ZOMBIE_STACK_SIZE.png)

4.继续执行以下命令，进行编译

```c
make BOARD=stm32f103-nano
```

5.如果编译正确无误，会产生XiZi_stm32f103-nano.elf、XiZi_stm32f103-nano.bin文件。其中XiZi_stm32f103-nano.bin需要烧写到设备中进行运行。

## 3. 烧写及执行

stm32f103-nano开发板内置板载st-link SWD下载接口，连接USB后便可使用st-flash工具进行烧写bin文件。

![stm32f103-nano](img/stm32f103-nano.png)

### 烧写工具

ARM：ST-LINK, stm32f103-nano 板卡内置st-link SWD, 因此无需额外外接工具，只需安装st-flash相关依赖驱动即可。

下载并以下执行命令以下命令安装st-link工具(本文使用v1.5.1版本)。

```
sudo apt install libusb-dev
sudo apt install libusb-1.0-0-dev
sudo apt  install cmake
cd stlink
make
cd build/Release && make install DESTDIR=_install
```

将生成的st-flash（在stlink/build/Release/bin文件夹下）复制到/usr/bin下就可使用了

代码根目录下执行st-flash工具烧录

```
sudo st-flash write build/XiZi_stm32f103-nano.bin 0x8000000
```

### 3.1 运行结果

如果编译 & 烧写无误，将会在串口终端上看到信息打印输出。

![terminal](img/terminal.png)
