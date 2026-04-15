cmake_minimum_required(VERSION 3.10)
project(depth-compute-opensource)

set (LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/../tofi/")

add_library(tofi_compute SHARED src/tofiCompute.cpp ../tofi/opencv_undistort.cpp)
target_include_directories(tofi_compute
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_HEADERS}>
    PRIVATE
        src/
)

add_library(tofi_config SHARED src/tofiConfig.cpp)
target_include_directories(tofi_config
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_HEADERS}>
        $<BUILD_INTERFACE:${CCB_HEADERS}>
    PRIVATE
        src/
)
