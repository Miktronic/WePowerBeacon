# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(beacon)

target_sources(app PRIVATE src/main.c src/app_fram.c src/app_encryption.c src/app_accel.c src/app_temp_pressure.c)
