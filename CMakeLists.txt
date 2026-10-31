# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(EXTRA_DTC_OVERLAY_FILE "dts/dts.overlay")
set(TOOLCHAIN_HAS_PICOLIBC ON CACHE BOOL "Force toolchain picolibc support" FORCE)
list(APPEND ZEPHYR_EXTRA_MODULES "/home/theorlangur/myapps/cpp/nrf/libs/aem13921")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aem13921_calibrator)

set(SRCS src/main.cpp)

target_sources(app PRIVATE ${SRCS})

target_compile_options(app PRIVATE -fno-omit-frame-pointer)

target_link_libraries(app PRIVATE c stdc++ supc++)
target_compile_options(app PRIVATE -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof -Wno-unused-variable -Wno-double-promotion)
target_compile_options(app PRIVATE -freflection)

