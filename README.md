# UVirtio: Enabling Ubiquitous Resource Sharing for RISC-V Industrial Edge Devices

This is the code repository for TACO (ACM Transactions on Architecture and Code Optimization) '26 manuscript. 

MULIANG SHOU, Shanghai Jiao Tong University, China
YUFAN JIANG, Shanghai Jiao Tong University, China
TIANLEI XIONG, Shanghai Jiao Tong University, China
CHENGGANG WU, Shanghai Jiao Tong University, China
WENDA TANG, Cloud Computing Research Institute, China Telecom, China
JIE WU, Cloud Computing Research Institute, China Telecom, China and Department of Computer and Information
Sciences, Temple University, United States
ZHENGWEI QI, Shanghai Jiao Tong University, China

## Quick Start

To run the K210 experiments on an emulator, install dependencies for compilation.
```
$ sudo apt install build-essential pkg-config
$ sudo apt install gcc make libncurses5-dev openssl libssl-dev bison flex libelf-dev autoconf libtool gperf libc6-dev  git
```

Download the `kconfig-frontends` toolchain and install.

```
mkdir kfrontends  && cd kfrontends
git clone https://gitlink.org.cn/xuos/kconfig-frontends.git

cd kconfig-frontends
 ./xs_build.sh
```

Also download the [GNU RISC-V Embedded GCC toolchain](https://xpack-dev-tools.github.io/riscv-none-elf-gcc-xpack/docs/install/) `riscv-none-embed-gcc`.

Clone this repository.
```
mkdir test  &&  cd test
git clone https://github.com/muliangshou/UVirtio.git
```

go to `Ubiquitous/XiZi/board/k210-emulator` and modify the contents of `board/k210-emulator/config.mk`.
```
export CROSS_COMPILE ?=[your_riscv-none-embed-gcc_chain_location]
```

Choose the kernel modules during compilation using menuconfig.
```
cd ./Ubiquitous/XiZi
make BOARD=k210-emulator distclean
make BOARD=k210-emulator menuconfig
```

Compile. `make BOARD=k210-emulator`. You will find .elf and .bin files if successful.

Now you can run the .elf on QEMU, for example `qemu-system-riscv64 -nographic -machine sifive_u -bios build/XiZi_k210-emulator.elf`.

If you have physical hardware, you can flash the .bin file onto the actual development board, like the `aiit-riscv64-board`. 

Install `k-flash` and, after you have the .bin file, run:
```
kflash -t build/XiZi_aiit-riscv64-board.bin -p /dev/ttyUSB0
```

## Project Structure
### 1. Other File Modifications

To implement the relevant functionalities of the designed Virtio device driver, combined with XiUOS's design, the files we modified are primarily located in this directory `\Ubiquitous\XiZi\`. However, some other files were also modified. The content of these files is not the focus of this document, so a brief explanation is uniformly provided here.

- `/arch/arm/cortex-m3/arch_interrupt.h`: Added relevant interrupt number macro definitions for using the cortex-m3's UART2.
- `/board/cortex-m3-emulator/connect_uart.c`, `/board/hifive1-emulator/third_party_driver/connect_uart.c`: These two files correspond to the ARM and RISC-V architectures, respectively, if you want to run QEMU simulations. Since there is no actual Virtio device in the simulation, we modified the serial device initialization part to implement the upper-layer Virtio device driver based on the vacant UART2. In these two files, we completed the creation, initialization, configuration (upper-layer calls) of the Virtio device, and related calls for interrupt handling.
- `/board/cortex-m3-emulator/board.c`, `/board/hifive1-emulator/board.c`: Added a function call `InstallVirtioSerial` in both files to start the previously created Virtio device.
- `/board/cortex-m3-emulator/board.h`, `/board/hifive1-emulator/board.h`: Added some macro definitions related to Virtio device naming in these two files. These names are used in the XiUOS device driver model to globally determine unique Bus, Device, and Driver.
- `/kernel/thread/console.c`: Implemented `InstallVirtioSerial` by mimicking the logic of XiUOS which uses a serial device to create a console. It uses the previously created Virtio device to create the corresponding serial device.
- `/resources/include/bus.h`: Added macro definitions related to the Virtio device driver.
- `/resources/include/device.h`: Added macro definitions, function declarations, and header file declarations related to the Virtio device driver.
- `/resources/Kconfig`: Added configuration options related to the Virtio device driver.
- `/resources/Makefile`: Added make content related to Virtio devices.
- `/tool/shell/letter-shell/cmd.c`: Added command-line instructions for testing and debugging during the development process.
- `/tool/shell/letter-shell/shell_port.c`: Modified `read_param.size` in the `userShellRead` function; otherwise, errors occur when using telnet port mapping for serial debugging, preventing correct input reading.

### 2. Virtio Common Interfaces

The common interfaces of the Virtio device driver implemented in this project mainly include structures and interfaces related to Virtqueue, register data structures related to Virtio-MMIO, and structures and interfaces related to VirtioBus, VirtioDevice, and VirtioDriver.

### 2.1 Virtqueue

Mainly includes `/resources/include/virt_ring.h` and `/resources/include/virtio.h`.

#### 2.1.1 `virt_ring.h`

This file contains the main part of Virtqueue, the ring buffer. Its main structures and macro definitions are implemented according to the Virtio 1.0 standard. The specific principles of Virtio technology will not be described again. It should be noted that for implementation considerations, we did not fully comply with the Virtio 1.0 standard and retained the older Vring structure.

- `struct vring`: The data structure for Vring.
    - `num`: The number of buffer descriptors.
    - `desc`: Pointer to the descriptor table.
    - `avail`: Pointer to the available descriptor ring buffer.
    - `used`: Pointer to the used descriptor ring buffer.
- `struct vring_desc`: Structure of a single descriptor.
    - `addr`: Memory address of the data corresponding to the descriptor.
    - `len`: Length of the data.
    - `flags`: Flags related to the descriptor.
    - `next`: Index of the next descriptor in the descriptor chain.
- `struct vring_avail`: Structure of the available descriptor ring buffer.
    - `flags`: Flags for the available descriptor table.
    - `idx`: End index of the available descriptor ring.
    - `ring`: Available descriptor ring.
- `struct vring_used`: Structure of the used descriptor ring buffer.
    - `flags`: Flags for the used descriptor table.
    - `idx`: End index of the used descriptor ring.
    - `ring`: Used descriptor ring.
- `struct vring_used_elem`: Structure of an entry in the used descriptor ring.
    - `id`: The descriptor index corresponding to a single element.
    - `len`: Length of the descriptor chain; it is 1 for a single descriptor.

Besides, there are some helper function interfaces related to Vring in this file, all implemented according to the Virtio standard, which will not be described here.

#### 2.1.2 `virtio.h`

Primarily defines the data structure for Virtqueue, related macro definitions, and function declarations. The structure of Virtqueue is as follows:

- `struct Virtqueue`
    - `struct vring vr`: The ring buffer structure declared in `vring.h`.
    - `name`: Name of the virtqueue, used to determine the unique vq.
    - `idx`: Index of the virtqueue, serving the same purpose as `name`.
    - `vq_list`: Doubly linked list for adding the virtqueue to the Virtio device's vq chain.
    - `free_head`: Index of the first free descriptor.
    - `num_free`: Number of free descriptors.
    - `num_added`: Number of newly added available descriptors.
    - `num_total`: Total number of descriptors.
    - `last_avail_idx`: Maximum index of available descriptors processed by the backend device.
    - `last_used_idx`: Maximum index of used descriptors processed by the frontend driver.
    - `dev`: Pointer to the Virtio device corresponding to the virtqueue.
    - `vring_mem`: Pointer to the memory region for the vring.
    - `vring_mem_size`: Size of the memory region for the vring.
    - `avail_sem`, `used_sem`: Semaphores used for debugging.

The related function interfaces are as follows:

- `VirtqueueAlloc`: Allocate and create a new virtqueue.
- `VqAllocDesc`: Allocate a new free descriptor from the descriptor table.
- `VqFreeDesc`: Free a descriptor and add it to the free descriptor chain.
- `VqAddAvail`: Add the descriptor index to the available ring buffer.
- `VqAddUsed`: Add the descriptor index to the used ring buffer.

The implementations of these functions are in `/resources/virtio/virtio.c`.

### 2.2 Virtio-MMIO

The main function of Virtio-MMIO can be summarized as mapping the configuration and operation registers of the Virtio device to memory addresses via memory mapping, then achieving configuration of the Virtio device by reading and writing memory addresses. This is primarily the mode adopted by Virtio technology on embedded devices. To supoprt simulation of MMIO operations on QEMU, we created similar register data structures in our implementation. The main content is in `/resources/include/virtio-mmio.h`, with the main structure `VirtioRegs`:

- `struct VirtioRegs`
    - `MagicValue`: The Magic Value specific to the Virtio device, a fixed value.
    - `Version`: The version of the Virtio specification followed.
    - `DeviceID`: The device number corresponding to the Virtio device.
    - `VendorID`: The manufacturer number of the Virtio device; has no practical meaning in the Virtio specification.
    - `DeviceFeatures`: Feature bits supported by the device.
    - `DeviceFeaturesSel`: Selection bit for the device feature set.
    - `DriverFeatures`: Feature bits supported by the driver.
    - `DriverFeaturesSel`: Selection bit for the driver feature set.
    - `QueueSel`: Virtqueue number corresponding to the virtqueue operation.
    - `QueueNumMax`: Maximum virtqueue size.
    - `QueueNum`: Actual virtqueue size used.
    - `QueueReady`: Identifier indicating virtqueue initialization is complete.
    - `QueueNotify`: Notify the device that the virtqueue has pending descriptors.
    - `InterruptStatus`: Interrupt status identifier.
    - `InterrupACK`: Identifier for the driver to acknowledge the interrupt.
    - `Status`: Device status.
    - `QueueDescLow`: Low bits of the descriptor table address.
    - `QueueDescHigh`: High bits of the descriptor table address.
    - `QueueAvailLow`: Low bits of the available ring address.
    - `QueueAvailHigh`: High bits of the available ring address.
    - `QueueUsedLow`: Low bits of the used ring address.
    - `QueueUsedHigh`: High bits of the used ring address.
    - `ConfigGeneration`: Configuration version.
    - `Config`: Device configuration space.

The order and length of each variable follow the Virtio 1.0 standard. For specific usage methods and details, please refer to the Virtio 1.0 standard document.

### 2.3 Virtio Bus, Virtio Device, and Virtio Driver

This part mainly aims to match XiUOS's own device driver model. By mimicking its usage, we created common Virtio device, driver, and bus structures, as well as related interface functions conforming to XiUOS specifications. It mainly includes three files: `/resources/virtio/bus_virtio.c`, `/resources/virtio/dev_virtio.c`, `/resources/virtio/drv_virtio.c`. The ownership of some functions in these files still needs further modification. Below, those marked as **(New)** are function interfaces not present in XiUOS itself, while those marked as **(Modified)** are function interfaces that have undergone significant changes.

#### 2.3.1 Virtio Bus

This is the content of `/resources/virtio/bus_virtio.c`, implementing some functions specified by XiUOS, but details still need further improvement. These functions include:

- `VirtioBusInit`: Initializes the data structure of the Virtio bus.
- `VirtioDriverInit`: Initializes the data structure of the Virtio driver.
- `VirtioReleaseBus`: Not yet implemented.
- `VirtioDriverAttachToBus`: Binds a Virtio device to the Virtio bus.

#### 2.3.2 Virtio Driver

This is the content of `/resources/virtio/drv_virtio.c`, implementing some functions specified by XiUOS, but details still need further improvement. These functions include:

- `VirtioDriverLinkInit`: Initializes the global Virtio driver linked list.
- `VirtioDeviceStatusInit` (New): Primarily performs some configuration checks required by the Virtio standard during Virtio driver startup, such as confirming MagicValue.
- `VirtioDriverFind`: Returns the specified Virtio driver by name.
- `VirtioDriverRegister`: Adds a Virtio driver to the global linked list.

It can be seen that the actual initialization content of the Virtio driver is included in `bus_virtio.c`, while this file mainly completes some global indexing work for Virtio drivers. This arrangement follows the implementation method of other XiUOS driver files.

#### 2.3.3 Virtio Device

This is the content of `/resources/virtio/dev_virtio.c`, implementing some functions specified by XiUOS. The main implementation refers to the serial device, but details still need further improvement. These functions include:

- `VirtioFeaturesSelect` (New): Used for the Virtio device driver to select and negotiate functions based on the functions actually provided (not fully implemented).
- `VirtioDevFindVq` (New): Returns the virtqueue of a specified Virtio device by name.
- `VirtioDevOpen` (Modified): Opens the Virtio device, performs some related initialization work according to the Virtio 1.0 specification, and further calls based on the specific Virtio device type.
- `VirtioDevClose`: Closes the Virtio device.
- `VirtioDevWrite` (Modified): Calls the respective Write interface based on the specific Virtio device type.
- `VirtioDevRead` (Modified): Calls the respective Read interface based on the specific Virtio device type.
- `VirtioSetIsr` (Modified): Has no content; the actual function is implemented by different Virtio devices.
- `VirtioRegisterInit` (Modified): Initializes the Virtio device, primarily focusing on Virtio specification content.
- `VirtioDeviceFind`: Finds the corresponding Virtio device by name.
- `VirtioDeviceRegister`: Initializes the Virtio device and adds it to the global linked list.
- `VirtioDeviceAttachToBus`: Binds the Virtio device to the Virtio bus.

### 3. Virtio Device Drivers

Based on the Virtio 1.0 specification, we implemented two devices, Virtio-blk and Virtio-net. Additionally, we implemented a simple Virtio-serial device based on the underlying serial device. Virtio-blk and Virtio-net lack actual underlying physical or virtual hardware support, so they only provide read/write interfaces adapted to the XiUOS device driver model for the upper layer. Internally, they implement virtqueue to complete the data interaction process between the Driver and the Device. As for Virtio-serial, it simply adds calls to the hardware interface (if emulated, this would be QEMU) at the lower layer of the Device.

#### 3.1 Virtio-serial

Mainly the content of `/resources/virtio/virtio_serial.c`, implementing the logic for the Virtio-serial device for the unified Virtio device interface.

The main operations of the Virtio-serial device are divided into read and write operations. The implementation logic for the read operation is:

> Read Interface -> Wait for virtqueue input ... Underlying Interrupt -> Interrupt Handler -> Place input content into virtqueue -> Notify read handler -> Read handler function reads input from virtqueue -> Return

The main implementation logic for the write operation is:

> Write Interface -> Put written content into virtqueue -> Notify device -> Device reads virtqueue content -> Call hardware-supported serial function to write content to the (virtual) serial device

- `VirtioSerialInit`: Initialization of the Virtio-serial device, including Virtio device feature negotiation, status setting, virtqueue creation, and pre-allocation of receive queue descriptors (for direct processing upon receiving serial input).
- `VirtioSerialRead`: Implements the read operation for the Virtio-serial device, waiting for the `used_sem` semaphore, then processing descriptors in the used ring upon acquisition.
- `VirtioSerialWrite`: Implements the write operation for the Virtio-serial device. Creates two tasks, `VirtioSerialDriver Handler` and `VirtioSerialDevice PutChar`, to simulate Virtio's operation mode as much as possible, while calling `VirtioSerialPutChar` to write the operation content into the virtqueue.
- `VirtioSerialTxUsed`: The function run by the `VirtioSerialDriver Handler` task. Loops to check for unprocessed used ring descriptors; if found, processes them and frees the descriptor space.
- `VirtioSerialDevicePutChar`: The function run by the `VirtioSerialDevice PutChar` task. Loops to check for unprocessed available ring descriptors; if found, calls the underlying `putchar` function interface, writes the descriptor content to the serial port, then adds the descriptor to the used ring.
- `VirtioSerialDeviceGetChar`: The actual content of the serial input interrupt handler. Reads the pre-allocated available descriptors in the receive queue, writes the serial input into them, places the descriptors into the used ring, and notifies the Read function via a semaphore.

#### 3.2 Virtio-blk

Mainly the content of `/resources/virtio/virtio_blk.c`, implementing the logic for the Virtio-blk device for the unified Virtio device interface.

The main operations of the Virtio-blk device are divided into read and write operations. The implementation logic for the read operation is essentially the same; the only difference is the final operation on the block device. The logic is:

> Read/Write Interface -> Organize read/write content (i.e., block device location, buffer address info for read/write operations) into a request format and place it into virtqueue -> Notify device -> Device reads virtqueue content -> Call hardware-supported function to complete read/write on the block device -> Notify frontend of operation completion, place descriptor into used ring -> Read data and operation result from used ring

- `VirtioBlkInit`: Initialization of the Virtio-blk device, including Virtio device feature negotiation, status setting, and virtqueue creation.
- `VirtioBlkRead`: Implements the read operation for the Virtio-blk device. Creates two tasks, `VirtioBlkDriverHandler` and `VirtioBlkDevice`, to simulate Virtio's operation mode as much as possible, while calling `VirtioBlkSubmit` to write the read operation content as a read request into the virtqueue; waits for the return of a semaphore (indicating operation completion).
- `VirtioBlkWrite`: Implements the write operation for the Virtio-blk device. Creates two tasks, `VirtioBlkDriverHandler` and `VirtioBlkDevice`, to simulate Virtio's operation mode as much as possible, while calling `VirtioBlkSubmit` to write the write operation content as a write request into the virtqueue; waits for the return of a semaphore (indicating operation completion).
- `VirtioBlkSubmit`: Organizes read/write requests into a descriptor chain format and places them into the available descriptor ring.
- `VirtioBlkIsr`: The function run by `VirtioBlkDriverHandler`. Loops to check for unprocessed used descriptors, processes them, and frees the descriptors in the used ring.
- `VirtioBlkDeviceHandleReq`: The function run by the `VirtioBlkDevice` task. Loops to check for unprocessed available descriptors, processes them based on the read/write operation type, and adds the descriptors for completed operations to the used ring.

#### 3.3 Virtio-net

Mainly the content of `/resources/virtio/virtio_net.c`, implementing the logic for the Virtio-net device for the unified Virtio device interface.

The main operations of the Virtio-net device are divided into read and write operations. The implementation logic for the read operation is:

> Read Interface -> Wait for virtqueue input ... Underlying Interrupt -> Interrupt Handler -> Place input content into virtqueue -> Notify read handler -> Read handler function reads input from virtqueue -> Return

The main implementation logic for the write operation is:

> Write Interface -> Put written content into virtqueue -> Notify device -> Device reads virtqueue content -> Call the underlying serial port function (which would be a network packet sending interface in real use) to write the content to the serial port

When there is a the lack of a real network device, UART2 is used for underlying operations to simulate the network device.

- `VirtioNetInit`: Initialization of the Virtio-net device, including Virtio device feature negotiation, status setting, virtqueue creation, and pre-allocation of receive queue descriptors (for direct processing upon receiving serial input).
- `VirtioNetRead`: Implements the read operation for the Virtio-net device, waiting for the `used_sem` semaphore, then processing descriptors in the used ring upon acquisition.
- `VirtioNetWrite`: Implements the write operation for the Virtio-net device. Creates two tasks, `VirtioNetDeviceTx` and `VirtioNetTxHandler`, to simulate Virtio's operation mode as much as possible, while calling `VirtioNetAddTxPacket` to write the write operation content into the virtqueue.
- `VirtioNetAddTxPacket`: Organizes the content of the write operation into a packet format and places it into the available ring as a descriptor chain.
- `VirtioNetTxUsed`: The function run by the `VirtioNetTxHandler` task. Loops to check for unprocessed used descriptors, processes them, and frees the descriptors in the used ring.
- `VirtioNetDeviceTx`: The function run by the `VirtioNetDeviceTx` task. Loops to check for unprocessed available ring descriptors; if found, calls the underlying `putchar` function interface (which should be a network packet sending interface in real use), writes the descriptor content to the serial port, then adds the descriptor to the used ring.
- `VirtioNetDeviceRx`: The actual content of the serial input interrupt handler (which should be the network packet receive interrupt handler in real use). Reads the pre-allocated available descriptors in the receive queue, writes the serial input into them, places the descriptors into the used ring, and notifies the Read function via a semaphore.

## TODO
- [x] Add project structure.
- [x] Add instructions for quick start.
- [ ] Upstream to current XiUOS version.
- [ ] Update the implementation of device aggregation to the newest branch.
