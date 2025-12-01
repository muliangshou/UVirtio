#切换到root目录
cd ../../
echo $(pwd)
git submodule init
git submodule update Ubiquitous/RT-Thread_Fusion_XiUOS/rt-thread
git submodule update Ubiquitous/RT-Thread_Fusion_XiUOS/aiit_board/k210/kendryte-sdk/kendryte-sdk-source