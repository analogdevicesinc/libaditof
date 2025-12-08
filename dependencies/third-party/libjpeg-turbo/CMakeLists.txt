cmake_minimum_required(VERSION 3.12)
project(libjpeg-turbo-wrapper)

include(ExternalProject)

set(JPEG_TURBO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libjpeg-turbo-src)
set(JPEG_TURBO_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/inst)
set(JPEG_TURBO_INCLUDE_DIR ${JPEG_TURBO_INSTALL_DIR}/include)
set(JPEG_TURBO_PREFIX_DIR ${CMAKE_CURRENT_BINARY_DIR}/ep)

# Set library path and name based on platform
if(WIN32)
    set(JPEG_TURBO_LIB_DIR ${JPEG_TURBO_INSTALL_DIR}/lib)
    set(JPEG_TURBO_LIB_NAME "turbojpeg-static.lib")
else()
    set(JPEG_TURBO_LIB_DIR ${JPEG_TURBO_INSTALL_DIR}/lib)
    set(JPEG_TURBO_LIB_NAME "libturbojpeg.a")
endif()

# Create the include directory upfront so CMake doesn't complain
file(MAKE_DIRECTORY ${JPEG_TURBO_INCLUDE_DIR})

# Platform detection for status message
if(WIN32)
    message(STATUS "libjpeg-turbo: Building for Windows x86-64 with SSE/AVX SIMD support")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
    message(STATUS "libjpeg-turbo: Building for ARM64 with NEON SIMD support")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    message(STATUS "libjpeg-turbo: Building for x86-64 with SSE/AVX SIMD support")
else()
    message(STATUS "libjpeg-turbo: Building for ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Use local source directory instead of downloading
ExternalProject_Add(
    jpegturbo
    PREFIX ${JPEG_TURBO_PREFIX_DIR}
    SOURCE_DIR ${JPEG_TURBO_SOURCE_DIR}
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${JPEG_TURBO_INSTALL_DIR}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DENABLE_SHARED=OFF
        -DENABLE_STATIC=ON
        -DWITH_TURBOJPEG=ON
        -DWITH_JPEG8=ON
        -DWITH_JAVA=OFF
        -DWITH_SIMD=ON
    BUILD_BYPRODUCTS
        ${JPEG_TURBO_LIB_DIR}/${JPEG_TURBO_LIB_NAME}
)

# Create an imported library target
add_library(jpeg-turbo STATIC IMPORTED GLOBAL)
add_dependencies(jpeg-turbo jpegturbo)

set_target_properties(jpeg-turbo PROPERTIES
    IMPORTED_LOCATION "${JPEG_TURBO_LIB_DIR}/${JPEG_TURBO_LIB_NAME}"
    INTERFACE_INCLUDE_DIRECTORIES "${JPEG_TURBO_INCLUDE_DIR}"
)

# For multi-config generators (Visual Studio), set IMPORTED_LOCATION for all configs
foreach(CONFIG_TYPE Release Debug RelWithDebInfo MinSizeRel)
    set_property(TARGET jpeg-turbo PROPERTY
        IMPORTED_LOCATION_${CONFIG_TYPE} "${JPEG_TURBO_LIB_DIR}/${JPEG_TURBO_LIB_NAME}"
    )
endforeach()
