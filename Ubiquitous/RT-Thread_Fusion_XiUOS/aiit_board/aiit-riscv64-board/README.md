## aiit-riscv64-board编译说明

编译aiit-riscv64-board，需要有RT-Thread的代码，因为aiit-riscv64-boar的sdk是以软件包方式，更新软件包在~/yourdir/xiuos/目录执行以下语句：
    
    git submodule init
    git submodule update Ubiquitous/RT_Thread/rt-thread
    git submodule update Ubiquitous/RT_Thread/aiit_board/aiit-riscv64-board/kendryte-sdk/kendryte-sdk-source 

如果在Linux平台下，可以先执行

    scons --menuconfig

退出后执行编译：

    scons

编译工具链安装参考：xiuos/Ubiquitous/XiZi/board/aiit-riscv64-board/README.md