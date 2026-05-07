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

Download the kconfig-frontends toolchain and install.

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

go to `Ubiquitous/XiZi/board/k210-emulator` and modify the contents of board/k210-emulator/config.mk.
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


## TODO
- [x] Add project structure.
- [x] Add instructions for quick start.
- [ ] Upstream to current OS version.
- [ ] Update the implementation of device aggregation to the newest branch.
- [ ] Release K210 plugin on RT-Thread.
