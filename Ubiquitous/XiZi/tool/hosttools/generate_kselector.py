#! /usr/bin/python3

import os

ubiquitous_dir = os.environ.get('UBIQUITOUS_ROOT')
bsp_dir = os.environ.get('BSP_ROOT')
kernel_names = []

def get_kernel_names():
    for d in os.scandir(ubiquitous_dir):
        if d.is_dir() and d.name!= "feature_yaml":
            kernel_names.append(d.name)

def generate_features():
    with open(f"{ubiquitous_dir}/Kselector_features","w") as f:
        # f.write("")
        pass

def generate_params():
    with open(f"{ubiquitous_dir}/Kselector_params","w") as f:
        # f.write("")
        pass

def generate():
    get_kernel_names()
    generate_features()
    generate_params()

    template = r"""
mainmenu "Ubiquitous Kernel Selector"

config BSP_DIR
    string
    option env="BSP_ROOT"
    default "."

config KERNEL_DIR
    string
    option env="KERNEL_ROOT"
    default "../.."

config UBIQUITOUS_DIR
    string
    option env="UBIQUITOUS_ROOT"
    default "../../.."

source "$UBIQUITOUS_DIR/Kselector_features"

source "$UBIQUITOUS_DIR/Kselector_params"

config MANUALLY_SELECT
    bool "Manually select a kernel"
    default n
if MANUALLY_SELECT
menu "Required Kernel"
choice
    prompt "Select OS Kernel"
    default SELECT_XIUOS

    """

    for kernel_name in kernel_names:     
        template += f"\tconfig SELECT_{kernel_name.upper()}\n"
        template += f'\t\tbool "select {kernel_name}"\n'

    # the last `\n` is very important, otherwise, it cannot be recognized as valid Kconfig file
    template += "endchoice\nendmenu\nendif\n" 

    with open(f"{bsp_dir}/Kselector", "w") as f:
        f.write(template)

if __name__ == '__main__':
    generate()