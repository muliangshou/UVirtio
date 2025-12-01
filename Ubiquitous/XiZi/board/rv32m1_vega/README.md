# 从零开始构建矽璓工业物联操作系统：使用risc-v架构的rv32m1_vega 开发板

[XiUOS](http://xuos.io/) (X Industrial Ubiquitous Operating System) 矽璓工业物联操作系统是一款面向工业物联场景的泛在操作系统，来自泛在操作系统研究计划。所谓泛在操作系统(UOS: Ubiquitous Operating Systems)，是支持互联网时代人机物融合泛在计算应用模式的新型操作系统，是传统操作系统概念的泛化与延伸。在泛在操作系统技术体系中，不同的泛在计算设备和泛在应用场景需要符合各自特性的不同UOS，XiUOS即是面向工业物联场景的一种UOS，主要由一个极简的微型实时操作系统(RTOS)内核和其上的智能工业物联框架构成，支持工业物联网(IIoT: Industrial Internet of Things)应用。


## 开发环境搭建

### 推荐使用：

**操作系统：** ubuntu18.04 [https://ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)

**开发工具推荐使用 VSCode   ，VScode下载地址为：** VSCode  [https://code.visualstudio.com/](https://code.visualstudio.com/)，推荐下载地址为 [http://vscode.cdn.azure.cn/stable/3c4e3df9e89829dce27b7b5c24508306b151f30d/code_1.55.2-1618307277_amd64.deb](http://vscode.cdn.azure.cn/stable/3c4e3df9e89829dce27b7b5c24508306b151f30d/code_1.55.2-1618307277_amd64.deb)

### 依赖包安装：

```
$ sudo apt install build-essential pkg-config
$ sudo apt install gcc make libncurses5-dev openssl libssl-dev bison flex libelf-dev autoconf libtool gperf libc6-dev  git
```

**源码下载：** XiUOS [https://www.gitlink.org.cn/xuos/xiuos](https://www.gitlink.org.cn/xuos/xiuos)

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

![vscode](img/vscode.jpg)
  
### 裁减配置工具的下载

裁减配置工具：

**工具地址：** kconfig-frontends [https://www.gitlink.org.cn/xuos/kconfig-frontends](https://www.gitlink.org.cn/xuos/kconfig-frontends)

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

RISC-V: riscv-none-embed-，默认安装到Ubuntu的/opt/，下载并解压。[下载网址 https://open-isa.org/downloads/]
![gnu](img/riscv_gnu.png)



将上述解压的编译工具链的路径添加到board/rv32m1_vega/config.mk文件当中，例如：

```
export CROSS_COMPILE ?=/opt/gnu-mcu-eclipse/riscv-none-gcc/8.2.0-2.1-20190425-1021/bin/riscv-none-embed-
```

若已存在`export CROSS_COMPILE ?=xxxx`   应该将原有的语句注释，再写入上面的语句。

# 在rv32m1_vega board 上创建第一个应用

## 1.rv32m1_vega board 简介

| 硬件 | 描述 |
| -- | -- |
|芯片型号| RV32M1 |
|架构| RV32IMAC |
|主频| 48MHz |
|片内SRAM| 1.25 MB Flash 384 KB SRAM |
| 外设 | BLK FSK/GFSK |

XiUOS板级当前支持使用UART。

## 2. 代码编写与编译说明

编辑环境：`VScode`

编译工具链：`riscv-none-embed-gcc`

使用`VScode`打开工程的方法有多种，本文介绍一种快捷键，在项目目录下将`code .`输入终端即可打开目标项目

修改`APP_Framework/Applications`文件夹下`main.c`

在输出函数中写入  Hello, world! \n 完成代码编辑。


编译步骤：

1.在VScode终端下执行以下命令，生成配置文件

```
cd ./Ubiquitous/XiZi
make BOARD=rm32v1_vega distclean
make BOARD=rm32v1_vega menuconfig
```

2.在menuconfig界面配置需要关闭和开启的功能，按回车键进入下级菜单，按Y键选中需要开启的功能，按N键选中需要关闭的功能，配置结束后选择Exit保存并退出

![menuconfig](img/menuconfig.png)

3.继续执行以下命令，进行编译

```
make BOARD=rv32m1_vega
```

4.如果编译正确无误，build文件夹下会产生XiZi_rv32m1_vega.elf、XiZi_rv32m1_vega.bin文件。

>注：最后可以执行以下命令，清除配置文件和编译生成的文件

```
make BOARD=rv32m1_vega distclean
```

## 3. 烧写及调试执行
rv32m1_vega开发板启动模式说明:参考文档[RV32M1_VEGA_Quick_Start_Guide.pdf](./doc/RV32M1_VEGA_Board_User_Guide.pdf)
![openocd](./img/multicore.jpg)


### 3.1 openocd gdb 调试方法
请使用JLink接入到RV32M1_VEGA开发板的RISC-V核的JTAG接口上，同时把JLink在PC上的驱动更改为WinUSB模式。JTAG接口位于RV32M1芯片和天线座子旁边，小的20pin JTAG接口。
参考文档：[RV32M1_VEGA_Quick_Start_Guide.pdf](./doc/RV32M1_VEGA_Quick_Start_Guide.pdf)

rv32m1_vega支持openocd，可以通过openocd和gdb进行调试。
调试需要下载openocd和sdk,下载配置方法参见以下文档：
https://github.com/open-isa-org/open-isa.org/blob/master/RV32M1_Vega_Develop_Environment_Setup.pdf

openocd安装完成以后，按照如下步骤进行调试：

1、进入xiuos目录路径下
```
cd ~/xiuos/Ubiquitous/XiZi 
```

2、编译生成elf文件


3、使用USB先和串口连接好开发板，进入openocd目录下，再在当前终端输入
```
cd /vega_rv32/sdk

Openocd -f <install_dir>\boards\rv32m1_vega\vega_ri5cy.cfg
```
在当前终端连接openocd，连接如下图所示：
![openocd](./img/openocd.png)

4、打开一个新的终端，输入以下命令打开终端串口：
```
sudo apt install screen
screen /dev/ttyUSB0 115200
```

5、打开一个新的终端，进入编译生成的elf路径,输入例如：
```
riscv-none-embed-gdb build/XiZi_rv32m1_vega.elf -ex "target remote localhost:3333"
```
结果如下图所示：
![gdb](./img/gdb_load.png)

6、再输入load，最后输入continue命令即可在串口终端看到系统运行界面，如下图所示：
![terminal](./img/terminal.png)


### 3.2 板载的串口调试
1、Windows 需要安装jlink驱动
2、开发板需要修改固件为RISCV启动，具体参考：
https://www.cnblogs.com/whik/p/10952292.html

https://open-isa.cn/community/topic/%E7%BB%87%E5%A5%B3%E6%98%9F%E5%BC%80%E5%8F%91%E6%9D%BF%E8%B0%83%E8%AF%95%E5%99%A8%E5%8D%87%E7%BA%A7%E4%B8%BAJlink%E5%9B%BA%E4%BB%B6/