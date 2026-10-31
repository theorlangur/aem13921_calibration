#!/bin/bash
source /home/theorlangur/ncs/toolchains/fbf7391cab/gcc16_env.sh
west build --build-dir build . --pristine \
    --board orlangur_ezurio_nrf54l15/nrf54l15/cpuapp -- \
    -DBOARD_ROOT=~/myapps/cpp/nrf \
    -DCONF_FILE="prj.conf config/cpp_gcc16.conf config/nrf54l15.conf config/bt.conf config/log.conf" \
    -DZEPHYR_TOOLCHAIN_VARIANT=cross-compile \
    -DNCS_TOOLCHAIN_VERSION=3.4.0 \
    -Dmcuboot_NCS_TOOLCHAIN_VERSION=3.4.0 \
    -DCMAKE_TOOLCHAIN_FILE=/home/theorlangur/ncs/toolchains/fbf7391cab/gcc16.cmake \
    -DTOOLCHAIN_HAS_PICOLIBC=ON

