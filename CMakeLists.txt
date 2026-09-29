# VS Code / ESP-IDF 插件从仓库根目录构建。
# 源码在 software/ 。不要把 software/ 整目录加进 EXTRA_COMPONENT_DIRS。

cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET esp32c3 CACHE STRING "ESP-IDF target")

set(SDKCONFIG_DEFAULTS
    "${CMAKE_CURRENT_LIST_DIR}/software/sdkconfig.defaults"
)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/software/main"
    "${CMAKE_CURRENT_LIST_DIR}/software/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ff_imu_handle)
