# 基于rt-thread的矽璓工业物联操作系统XiUOS

矽璓工业物联操作系统XiUOS主要分为系统层 、框架层、和应用层。其中系统层支持XIUOS、Nuttx、rt-thread三个操作系统，该目录主要内容是基于**rt-thread的系统层。**

## 目录内容

```
xiuos/Ubiquitous/RT-Thread_Fusion_XiUOS
    ├── README.md    
    ├── aiit_board              板级支持包
        |──stm32f407_core
        |──k210
    ├── app_match_rt-thread     WiFi、Camera 等适配rt-thread的用户驱动
    └── rt-thread               rt-thread 代码
```

## 支持平台

RT-Thread_Fusion_XiUOS/aiit_board 目前主要支持平台：stm32f407_core，k210,aiit-riscv64-board,stm32h743_openmv_h7plus,xidatong.

RT-Thread_Fusion_XiUOS/app_match_rt-thread  目前主要支持 WiFi 和 Camera，其他驱动复用 RT-Thread 的内容。如果rt-thread官方仓库驱动不满足用户使用要求，如果用户增加相关驱动可在此目录。

RT-Thread_Fusion_XiUOS/rt-thread 使用 RT-Thread 作为系统基础设施，提供底层支持。

## 使用

运行以下指令下载代码、编译运行：

```
# 下载代码
# 进入xiuos目录下载更新子模块(包括RT-thread 和 K210 SDK（如果用到K210 需更新kendryte-sdk）)执行以下三条命令或者直接执行当前目录下的download.sh脚本命令
git submodule init
git submodule update Ubiquitous/RT-Thread_Fusion_XiUOS/rt-thread
git submodule update Ubiquitous/RT-Thread_Fusion_XiUOS/aiit_board/k210/kendryte-sdk/kendryte-sdk-source

# 进入 xiuos/Ubiquitous/RT-Thread_Fusion_XiUOS/aiit_board/stm32f407_core 配置 Kconfig
scons --menuconfig
# 编译
scons
# 烧录镜像
st-flash write rtthread.bin 0x8000000
```

同时也可以支持windows开发环境进行上述命令进行编译，需安装env插件，详细介绍可以参照[rt-thread官方资料](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/application-note/setup/standard-project/an0017-standard-project?id=%e4%bd%bf%e7%94%a8-env-%e5%88%9b%e5%bb%ba-rt-thread-%e9%a1%b9%e7%9b%ae%e5%b7%a5%e7%a8%8b
)。

