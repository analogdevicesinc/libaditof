cmake_minimum_required(VERSION 3.15)
project(network_discovery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Server executable
if (ON_TARGET)
    add_library(${PROJECT_NAME} STATIC
        platform.cpp
        network_utils.cpp
        protocol.cpp
        discovery_server.cpp
    )
endif()

# Client executable
if (NOT ON_TARGET)
    add_library(${PROJECT_NAME} STATIC
        platform.cpp
        network_utils.cpp
        protocol.cpp
        discovery_client.cpp
    )
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    PRIVATE
        .
)