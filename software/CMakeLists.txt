# ESP32-C3FH4 固件的可选独立构建入口。
# 推荐从仓库根目录构建；在本目录执行 idf.py 也使用同一套源码与配置。

cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET esp32c3 CACHE STRING "ESP-IDF target")

set(SDKCONFIG_DEFAULTS
    "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ff_imu_handle)
