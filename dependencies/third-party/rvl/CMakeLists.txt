cmake_minimum_required(VERSION 3.0)
project(rvl)

#-------------------------------------------------------------------------------
# Platform detection and SIMD selection
#-------------------------------------------------------------------------------
# RVL_FORCE_SCALAR: Force scalar implementation (OFF by default)
# Auto-detects: ARM64 -> NEON, x86-64 -> SSE2/AVX, otherwise -> scalar
#-------------------------------------------------------------------------------

option(RVL_FORCE_SCALAR "Force scalar RVL implementation (disable SIMD)" OFF)
option(RVL_FAST_MODE "Use fast compression (less compression, more speed)" OFF)

# Detect platform
set(RVL_IMPL "scalar")
set(RVL_SIMD_DEFINES "")

if(NOT RVL_FORCE_SCALAR)
    # Check for ARM64 / NEON
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
        set(RVL_IMPL "neon")
        set(RVL_SIMD_DEFINES "RVL_USE_NEON=1")
        message(STATUS "RVL: Detected ARM64, using NEON optimized implementation")
    # Check for x86-64 / SSE
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64" OR 
           (WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8))
        set(RVL_IMPL "sse")
        set(RVL_SIMD_DEFINES "RVL_USE_SSE=1")
        message(STATUS "RVL: Detected x86-64, using SSE optimized implementation")
    else()
        message(STATUS "RVL: Unknown platform (${CMAKE_SYSTEM_PROCESSOR}), using scalar implementation")
    endif()
else()
    message(STATUS "RVL: Forced scalar implementation (RVL_FORCE_SCALAR=ON)")
endif()

# Select source file based on implementation
if(RVL_IMPL STREQUAL "neon")
    add_library(${PROJECT_NAME} STATIC
        RVL_neon.cpp
        RVL_internal.h
    )
elseif(RVL_IMPL STREQUAL "sse")
    add_library(${PROJECT_NAME} STATIC
        RVL_sse.cpp
        RVL_internal.h
    )
else()
    add_library(${PROJECT_NAME} STATIC
        RVL_scalar.cpp
        RVL_internal.h
    )
endif()

# Add SIMD compile definitions
if(RVL_SIMD_DEFINES)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ${RVL_SIMD_DEFINES})
endif()

# Add fast mode definition (PUBLIC so it propagates to users of the library)
if(RVL_FAST_MODE)
    target_compile_definitions(${PROJECT_NAME} PUBLIC RVL_FAST_MODE=1)
endif()

# Enable SSE2 on MSVC for x86-64 (usually enabled by default, but be explicit)
if(RVL_IMPL STREQUAL "sse")
    if(MSVC)
        # MSVC: SSE2 is default for x64, but enable AVX if available
        target_compile_options(${PROJECT_NAME} PRIVATE /arch:AVX)
    else()
        # GCC/Clang: enable SSE2 and AVX
        target_compile_options(${PROJECT_NAME} PRIVATE -msse2 -mavx)
    endif()
endif()

if(NOT WIN32)
    set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    PRIVATE
        .
)
