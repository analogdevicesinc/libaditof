cmake_minimum_required(VERSION 3.0)
project(rvl)

# User-visible option with default ON
option(RVL_USE_NEON "Enable ARM64 optimized RVL implementation" ON)

# Override default automatically on Windows x86/x64
if (WIN32 AND NOT CMAKE_SYSTEM_PROCESSOR MATCHES "ARM64|aarch64|arm64")
    set(RVL_USE_NEON OFF CACHE BOOL "" FORCE)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "ARM64|aarch64|arm64")
    set(RVL_USE_NEON ON CACHE BOOL "" FORCE)
endif()


if(RVL_USE_NEON)
    add_library(${PROJECT_NAME} STATIC
        RVL_neon.cpp
        RVL_internal.h
    )
    target_compile_definitions(${PROJECT_NAME} PRIVATE RVL_USE_NEON=1)
else()
    add_library(${PROJECT_NAME} STATIC
        RVL_scalar.cpp
        RVL_internal.h
    )
endif()

if (NOT WIN32) 
    set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    PRIVATE
        .
)
