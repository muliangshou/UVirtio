# Virtio for XiUOS

### 1. Introduction

基于 XiUOS 的设备驱动模型和 Virtio 1.0 标准，开发了本设备驱动组件，旨在在 XiUOS 中能够发挥 Virtio 的技术优势，更好地利用虚拟设备资源。

由于技术和开发条件的限制，目前我们所实现的 Virtio 设备驱动仅仅是基于 XiUOS 设备驱动模型的一层封装，即符合 XiUOS 设备驱动接口，内部采用 Virtio 1.0 标准所规定的设备驱动前后端解耦、数据面控制面分离的接口，模拟了 Virtio-MMIO 和相应的设备寄存器负责控制面，创建了共享队列 Virtqueue 进行数据面交互。但实际上并未使用真实的 Virtio 后端设备或虚拟设备，更多的功能有待后续的进一步开发实现。

我们按照 Virtio 1.0 标准的部分要求，实现了 Virtio-blk、Virtio-net 以及一个基于串口设备的 Virtio-serial 设备。实现了 Virtio 设备驱动所共用的接口和数据结构，尽可能在利用 XiUOS 本身功能的前提下，按照 Virtio 设备的使用方法和逻辑完成 I/O 操作。

### 2. Virtio 模型

我们所设计的 Virtio 设备驱动模型在 XiUOS 中大致可以表示为如下形式：

—— 应用层 ——

—— 前端驱动层 ——

—— Virtio 接口层 ——

—— 后端设备层 ——

—— 硬件层 ——

我们在本 repo 中所实现的内容就大致包括中间三层，由于出于开发初期，为了尽量贴近 XiUOS 本身的设备驱动层次，所以本目录下的代码并没有形成严格的层次关系，后续的内容讲对此进行简单的阐述。

### 3. 其他文件修改

为了实现所设计的 Virtio 设备驱动的相关功能，结合 XiUOS 的设计，我们修改的文件主要都在本目录下，但也修改了部分其他文件，这部分文件的内容不是本文的重点，因此统一在此处进行简要说明

1. ```/arch/arm/cortex-m3/arch_interrupt.h```：为了使用 cortex-m3 的 UART2，添加了相关的中断号宏定义；
2. ```/board/cortex-m3-emulator/connect_uart.c```、```/board/hifive1-emulator/third_party_driver/connect_uart.c``` ：本项目目前是基于 QEMU 模拟的开发板实现的，这两个文件分别对应的 ARM 与 RISC-V 架构。由于尚且没有实际的 Virtio 设备支持，我们修改了其中串口设备初始化的部分，基于尚未使用的 UART2 实现上层的 Virtio 设备驱动。在这两个文件内完成 Virtio 设备的创建、初始化和配置（的上层调用）以及中断处理的相关调用；
3. ```/board/cortex-m3-emulator/board.c```、```/board/hifive1-emulator/board.c```：这两个文件中都新增了函数调用 ```InstallVirtioSerial``` ，开启之前创建的 Virtio 设备；
4. ```/board/cortex-m3-emulator/board.h```、```/board/hifive1-emulator/board.h```：这两个文件中新增了 Virtio 设备命名相关的一些宏定义，这些命名在 XiUOS 设备驱动模型中用于全局确定唯一的 `Bus`、`Device` 和 `Driver`；
5. ```/kernel/thread/console.c```：实现了 `InstallVirtioSerial` 仿照 XiUOS 使用串口设备创建控制台 `console` 的逻辑，使用之前创造的 Virtio 设备创建对应的串口设备；
6. `/resources/include/bus.h`：添加了 Virtio 设备驱动相关的宏定义；
7. `/resources/include/device.h`：添加了 Virtio 设备驱动相关的宏定义、函数声明和头文件声明；
8. ```/resources/Kconfig```：添加了 Virtio 设备驱动相关的配置选项；
9. ```/resources/Makefile```：添加了 Virtio 设备相关的 make 内容；
10. `/tool/shell/letter-shell/cmd.c`：新增了命令行指令，用于开发过程中的测试与调试；
11. ```/tool/shell/letter-shell/shell_port.c```：修改了 `userShellRead` 函数的 `read_param.size`，否则在使用 telnet 端口映射串口进行调试的时候会发生错误，不能正确读取输入；

### 4.Virtio 通用接口

本项目所实现的 Virtio 设备驱动的通用接口主要包括 Virtqueue 相关的结构和接口、Virtio-MMIO 相关的寄存器数据结构、`VirtioBus`、`VirtioDevice`、`VirtioDriver`相关结构和接口。

##### 4.1 Virtqueue

主要包括 `/resources/include/virt_ring.h` 和 `/resources/include/virtio.h` 

##### 4.1.1 `virt_ring.h` 

本文件内包含了 Virtqueue 的主要部分，即环形缓冲区。其主要的结构和宏定义都是按照 Virtio 1.0 标准所实现的，对于 Virtio 技术具体的原理不再叙述。需要注意的是，出于实现的考虑，没有完全按照 Virtio 1.0 的标准，仍然保留了更早的 `Vring` 结构。

1. `struct vring`： 就是 Vring 对应的数据结构

   ​	——`num` ：缓冲区的描述符数量；

   ​	——`desc` ：指向描述符表的指针；

   ​	——`avail` ：指向可用描述符环形缓冲区的指针；

   ​	——`used`  ：指向已用描述符环形缓冲区的指针；

2. `struct vring_desc` ：单个描述符的结构

   ​	——`addr` ：描述符对应的数据的内存地址；

   ​	——`len` ：数据的长度；

   ​	——`flags` ：描述符相关的 flag；

   ​	——`next` ：描述符链的下一个描述符下标；

3. `struct vring_avail` ：可用描述符环形缓冲区结构

   ​	——`flags` ：可用描述符表的 flag；

   ​	——`idx` ：可用描述符环的末尾下标；

   ​	——`ring` ：可用描述符环

4. `struct vring_used` ：已用描述符环形缓冲区结构

   ​	——`flags` ：已用描述符表的 flag；

   ​	——`idx` ：已用描述符环的末尾下标；

   ​	——`ring` ：已用描述符环

5. `struct vring_used_elem` ：已用描述符环的条目结构

   ​	——`id` ：单个 element 所对应的描述符下标；

   ​	——`len` ：描述符链的长度，单个描述符则为 1；

除此之外，本文件中还有一些 Vring 相关的辅助函数接口，都是按照 Virtio 标准实现的，在此不做叙述。

##### 4.1.2 `virtio.h` 

主要定义了 Virtqueue 的数据结构，相关的宏定义和函数声明。`Virtqueue` 的结构如下：

`struct Virtqueue`

​	——`struct vring vr` ：即 `vring.h` 中声明的环形缓冲区结构；

​	——`name` ：virtqueue 的名称，用于确定唯一的 vq；

​	——`idx`：virtqueue 的 index，用处和 `name`  相同；

​	——`vq_list` ：双向链表，用于将 virtqueue 加入 Virtio 设备的 vq 链；

​	——`free_head` ：第一个空闲的描述符的下标；

​	——`num_free` ：空闲描述符的数量；

​	——`num_added` ：新增的可用描述符的数量；

​	——`num_total` ：总共的描述符数量；

​	——`last_avail_idx` ：后端设备已处理的最大可用描述符下标；

​	——`last_used_idx` ：前端驱动已处理的最大已用描述符下标；

​	——`dev` ：指向 virtqueue 对应的 Virtio 设备的指针；

​	——`vring_mem` ：`vring` 对应的内存区域指针；

​	—— `vring_mem_size` ：`vring` 对应的内存区域大小；

​	——`avail_sem` 、`used_sem` ：信号量，用于 debug；

相关的函数接口如下：

1. `VirtqueueAlloc` ：分配创建新的 virtqueue；
2. `VqAllocDesc` ：从描述符表中分配新的空闲描述符；
3. `VqFreeDesc` ：释放描述符，加入空闲描述符链；
4. `VqAddAvail` ：将描述符的下标加入到可用环形缓冲区；
5. `VqAddUsed` ：将描述符的下标加入到已用环形缓冲区；

这些函数的实现都在文件 `/resources/virtio/virtio.c` 中。

##### 4.2 Virtio-MMIO

Virtio-MMIO 的主要功能可以概括为通过内存映射的方式，将 Virtio 设备的配置、操作寄存器映射到内存地址，然后通过读写内存的方式实现 Virtio 设备的配置等工作，这主要是 Virtio 技术在嵌入式设备上采用的模式。由于暂时没有使用实际的 Virtio 设备，我们的实现中创建了类似的寄存器数据结构，模拟真实 Virtio 设备在使用的过程中的 MMIO 操作。主要的内容在文件 `/resources/include/virtio-mmio.h` 中，主要结构是 `VirtioRegs`：

`struct VirtioRegs`

​	——`MagicValue` ：Virtio 设备特有的 Magic Value，为固定值；

​	——`Version` ：所遵循的 Virtio 规范的版本；

​	——`DeviceID` ：Virtio 设备对应的设备号；

​	——`VendorID` ：Virtio 设备的生产厂商号；Virtio 规范中没有实际的意义；

​	——`DeviceFeatures` ：设备支持的功能标识；

​	——`DeviceFeaturesSel` ：设备功能集的选择位；

​	——`DriverFeatures` ：驱动支持的设备功能的标识；

​	——`DriverFeaturesSel` ：驱动使用的设备功能集的选择位；

​	——`QueueSel` ：virtqueue 操作对应的 virtqueue 编号；

​	——`QueueNumMax` ：最大的 virtqueue 大小；

​	——`QueueNum` ：实际使用的 virtqueue 大小；

​	——`QueueReady` ：virtqueue 初始化完成标识；

​	——`QueueNotify` ：通知设备该 virtqueue 有待处理的描述符；

​	——`InterruptStatus` ：中断状态标识符；

​	——`InterrupACK` ：驱动确认中断的标识符；

​	——`Status` ：设备状态；

​	——`QueueDescLow` ：描述符表地址的低位；

​	——`QueueDescHigh` ：描述符表地址的高位；

​	——`QueueAvailLow` ：可用环的地址低位；

​	——`QueueAvailHigh` ：可用环的地址高位；

​	——`QueueUsedLow` ：已用环的地址低位；

​	——`QueueUsedHigh` ：已用环的地址高位；

​	——`ConfigGeneration` ：配置版本；

​	——`Config` ：设备配置空间；

各个变量的顺序、长度都按照 Virtio 1.0 标准定义，其具体的使用方法和细节请见 Virtio 1.0 标准文档。

##### 4.3 Virtio Bus、Virtio Device 及 Virtio Driver

这部分的内容主要是为了匹配 XiUOS 本身的设备驱动模型，仿照其使用方法，创建了通用的 Virtio 设备、驱动和总线的结构，以及相关的符合 XiUOS 规范的接口函数，主要包括 `/resources/virtio/bus_virtio.c` 、`/resources/virtio/dev_virtio.c` 、`/resources/virtio/drv_virtio.c` 三个文件。其中部分函数的归属文件仍有待修改。下面标有 *新增* 的即是 XiUOS 本身没有的函数接口，标有 *修改* 的则是做出了比较大改动的函数接口。

##### 4.3.1 Virio Bus

即文件 `/resources/virtio/bus_virtio.c` 的内容，实现了 XiUOS 所规定的部分函数，但具体的细节还有待进一步的完善，这些函数包括：

1. `VirtioBusInit` ：初始化 Virtio 总线的数据结构；
2. `VirtioDriverInit` ：初始化 Virtio 驱动的数据结构；
3. `VirtioReleaseBus` ：*尚未实现*；
4. `VirtioDriverAttachToBus` ：将 Virtio 设备绑定到 Virtio 总线上；

##### 4.3.2 Virtio Driver

即文件 `/resources/virtio/drv_virtio.c` 的内容，实现了 XiUOS 所规定的部分函数，但具体的细节还有待进一步的完善，这些函数包括：

1. `VirtioDriverLinkInit` ：初始化全局 Virtio 驱动链表；
2. `VirtioDeviceStatusInit` ：（*新增*）主要是进行一些 Virtio 标准规定的 Virtio 驱动启动时的配置检查工作，如 MagicValue 的确认等；
3. `VirtioDriverFind` ：根据名称返回指定的 Virtio 驱动；
4. `VirtioDriverRegister` ：将 Virtio 驱动添加到全局链表中；

可以看到，Virtio 驱动实际的初始化内容被包括在了 `bus_virtio.c` 文件，而本文件主要完成了一些 Virtio 驱动的全局索引工作，这是参考 XiUOS 其他的驱动文件实现方法安排的。

##### 4.3.3 Virtio Device

即文件 `/resources/virtio/dev_virtio.c` 的内容，实现了 XiUOS 所规定的部分函数，主要的实现参考了串口设备，但具体的细节还有待进一步的完善，这些函数包括：

1. `VirtioFeaturesSelect` ：（*新增*）用于 Virtio 设备驱动根据实际提供的功能进行功能选择和协商（并未真正实现）
2. `VirtioDevFindVq` ：（*新增*）通过名称返回指定 Virtio 设备的 virtqueue；
3. `VirtioDevOpen` ：（*修改*）打开 Virtio 设备，按照 Virtio 1.0 规范进行一些相关的初始化工作，会进一步根据具体的Virtio 设备类型进行调用；
4. `VirtioDevClose` ：关闭 Virtio 设备；
5. `VirtioDevWrite` ：（*修改*）根据具体的 Virtio 设备类型调用各自的 Write 接口；
6. `VirtioDevRead` ：（*修改*）根据具体的 Virtio 设备类型调用各自的 Write 接口；
7. `VirtioSetIsr` ：（*修改*）无内容，实际的功能由不同的 Virtio 设备具体实现；
8. `VirtioRegisterInit` ：（*修改*）初始化 Virtio 设备，主要是 Virtio 规范的内容；
9. `VirtioDeviceFind` ：通过名称找到对应的 Virtio 设备；
10. `VirtioDeviceRegister` ：初始化 Virtio 设备，并添加到全局链表中；
11. `VirtioDeviceAttachToBus` ：将 Virtio 设备与 Virtio 总线绑定；

### 5. Virtio 设备驱动

基于 Virtio 1.0 规范的内容，我们实现了 Virtio-blk 和 Virtio-net 两个设备，另外还基于底层的串口设备实现了简单的 Virtio-serial 设备。Virtio-blk 和 Virtio-net 并没有实际的底层物理硬件或虚拟硬件支持，所以仅仅是根据其使用特点，对上提供适配 XiUOS 设备驱动模型的读写接口，内部实现了 virtqueue 完成 Driver 和 Device 的数据交互的过程。而 Virtio-serial 则只是在 Device 的下层加上了对于（QEMU 模拟的）硬件接口的调用。

##### 5.1 Virtio-serial

主要是文件 `/resources/virtio/virtio_serial.c` 的内容，针对统一的 Virtio 设备接口，实现了对于 Virtio-serial 设备的逻辑。

Virtio-serial 设备的主要操作分为读写操作，读操作的实现逻辑主要是：

Read 接口->等待 virtqueue 输入 ... 底层中断->中断处理函数->将输入内容放入 virtqueue->通知 read handler ... Read handler 函数从 virtqueue 读取输入->返回

写操作的主要实现逻辑是：

Write 接口->将写的内容放入 virtqueue->通知 device -> device 读取 virtqueue 的内容->调用硬件支持的串口函数将内容写入（虚拟）串口设备

1. `VirtioSerialInit` ：Virtio-serial 设备的初始化，包括 Virtio 设备的功能协商、状态设置、virtqueue 的创建、接收队列的描述符预分配（方便接收到串口输入时直接进行处理）；
2. `VirtioSerialRead` ：Virtio-serial 设备的读操作实现，等待 `used_sem` 信号量，获得后进行处理已用环中的描述符；
3. `VirtioSerialWrite` ：Virtio-serial 设备的写操作实现，创建了 `VirtioSerialDriver Handler` 和 `VirtioSerialDevice PutChar` 两个任务，用于尽量模拟 Virtio 的运行方式，同时调用 `VirtioSerialPutChar` 将写操作的内容写入 virtqueue；
4. `VirtioSerialTxUsed` ：任务 `VirtioSerialDriver Handler` 所运行的函数，循环检查是否有未处理的已用环描述符，有则处理并释放描述符的空间；
5. `VirtioSerialDevicePutChar` ：任务 `VirtioSerialDevice PutChar` 所运行的函数，循环检查是否有未处理的可用环描述符，有则调用底层 `putchar` 函数接口，将描述符的内容写入串口，然后将描述符加入已用环；
6. `VirtioSerialDeviceGetChar` ：串口输入的中断处理函数实际内容，读取接收队列预先准备好的可用描述符，将串口的输入写入并将描述符放入已用环，同时通过信号量的方式通知 Read 函数；

##### 5.2 Virtio-blk

主要是文件 `/resources/virtio/virtio_blk.c` 的内容，针对统一的 Virtio 设备接口，实现了对于 Virtio-blk 设备的逻辑。

Virtio-blk 设备的主要操作分为读写操作，读操作的实现逻辑其实本质上是相同的，唯一不同的是最终对块设备的操作，其逻辑主要是：

Read/Write 接口->以操作请求的形式，将读写的内容（即块设备的定位、读写操作的缓冲区地址信息）放入 virtqueue->通知 device -> device 读取 virtqueue 的内容->调用硬件支持的串口函数将完成对块设备的读写->通知前端操作完成，将描述符放入已用环->从已用环读取数据和操作结果

1. `VirtioBlkInit` ：Virtio-blk 设备的初始化，包括 Virtio 设备的功能协商、状态设置、virtqueue 的创建；
2. `VirtioBlkRead` ：Virtio-blk 设备的读操作实现，创建了 `VirtioBlkDriverHandler` 和 `VirtioBlkDevice` 两个任务，用于尽量模拟 Virtio 的运行方式，同时调用 `VirtioBlkSubmit` 将读操作的内容以读请求的形式写入 virtqueue；等待信号量的返回（标志着完成操作）；
3. `VirtioBlkWrite` ：Virtio-blk 设备的写操作实现，创建了 `VirtioBlkDriverHandler` 和 `VirtioBlkDevice` 两个任务，用于尽量模拟 Virtio 的运行方式，同时调用 `VirtioBlkSubmit` 将写操作的内容以写请求的形式写入 virtqueue；等待信号量的返回（标志着完成操作）；
4. `VirtioBlkSubmit` ：将读写请求组织为描述符链的形式，放入可用描述符环；
5. `VirtioBlkIsr` ：`VirtioBlkDriverHandler` 所运行的函数，循环检查是否有未处理的已用描述符，处理并释放已用环中的描述符；
6. `VirtioBlkDeviceHandleReq` ：`VirtioBlkDevice` 任务所运行的函数，循环检查是否有未处理的可用描述符，根据读写操作的类型进行处理，并将已完成的操作对于的描述符加入已用环；

##### 5.3 Virtio-net

主要是文件 `/resources/virtio/virtio_net.c` 的内容，针对统一的 Virtio 设备接口，实现了对于 Virtio-net 设备的逻辑。

Virtio-net 设备的主要操作分为读写操作，读操作的实现逻辑主要是：

Read 接口->等待 virtqueue 输入 ... 底层中断->中断处理函数->将输入内容放入 virtqueue->通知 read handler ... Read handler 函数从 virtqueue 读取输入->返回

写操作的主要实现逻辑是：

Write 接口->将写的内容放入 virtqueue->通知 device -> device 读取 virtqueue 的内容->调用硬件支持的串口函数将内容写入（虚拟）串口设备

需要注意的是，由于没有真实的网络设备，所以使用了串口 UART2 来进行底层的操作，模拟网络设备；

1. `VirtioNetInit` ：Virtio-net 设备的初始化，包括 Virtio 设备的功能协商、状态设置、virtqueue 的创建、接收队列的描述符预分配（方便接收到串口输入时直接进行处理）；
2. `VirtioNetRead` ：Virtio-net 设备的读操作实现，等待 `used_sem` 信号量，获得后进行处理已用环中的描述符；
3. `VirtioNetWrite` ：Virtio-net 设备的写操作实现，创建了 `VirtioNetDeviceTx` 和 `VirtioNetTxHandler` 两个任务，用于尽量模拟 Virtio 的运行方式，同时调用 `VirtioNetAddTxPacket` 将写操作的内容写入 virtqueue；
4. `VirtioNetAddTxPacket` ：将写操作的内容组织为 packet 的格式，以描述符链的形式放入可用环中；
5. `VirtioNetTxUsed` ：任务 `VirtioNetTxHandler` 所运行的函数，循环检查是否有未处理的已用描述符，处理并释放已用环中的描述符；
6. `VirtioNetDeviceTx` ：任务 `VirtioNetDeviceTx` 所运行的函数，循环检查是否有未处理的可用环描述符，有则调用底层 `putchar` 函数接口（真实使用中应该为网络包发生接口），将描述符的内容写入串口，然后将描述符加入已用环；
7. `VirtioNetDeviceRx` ：串口输入的中断处理函数实际内容（真实使用中应为网络包接收中断的处理函数），读取接收队列预先准备好的可用描述符，将串口的输入写入并将描述符放入已用环，同时通过信号量的方式通知 Read 函数；

