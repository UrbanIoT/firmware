#-------------------------------------------------------------------------------
# UrbanIoT Zephyr firmware
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD rpi_pico)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C VERSION 1.0.0)

configure_file(app_version.h.in ${CMAKE_BINARY_DIR}/app/include/app_version.h)
target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/app/include src)

add_subdirectory(src/lora)
add_subdirectory(src/gps)

file(GLOB app_src src/*.c)
target_sources(app PRIVATE ${app_src})
