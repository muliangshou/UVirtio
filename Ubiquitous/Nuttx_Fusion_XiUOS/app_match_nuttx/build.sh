#!/usr/bin/env bash

export current=$(pwd)
export nuttx=$current/..
export top=$current/../../..

cd $top
git submodule
git submodule init
git submodule update Ubiquitous/Nuttx_Fusion_XiUOS/apps
git submodule update Ubiquitous/Nuttx_Fusion_XiUOS/nuttx
cd $current

find $top -name Kconfig -exec dos2unix -q {} \;

cp -rf $current/nuttx $nuttx
cp -rf $current/apps $nuttx
cp -rf $nuttx/aiit_board/aiit-arm32-board  $nuttx/nuttx/boards/arm/stm32
cp -rf $nuttx/aiit_board/aiit-riscv64-board $nuttx/nuttx/boards/risc-v/k210
cp -rf $nuttx/aiit_board/xidatong $nuttx/nuttx/boards/arm/imxrt

cd ../nuttx
