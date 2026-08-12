cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0075 NEW)
project(dependecies)

add_subdirectory(adi/command_parser)

# Only build json-c if the parent project hasn't already defined the target
if(NOT TARGET json-c)
    add_subdirectory(third-party/json-c)
endif()

add_subdirectory(third-party/crc32)

# Suppress warnings from json-c headers
if(TARGET json-c-static)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(json-c-static PRIVATE
            -Wno-unused-function
            -Wno-unused-variable
            -Wno-unused-but-set-variable)
    elseif(MSVC)
        target_compile_options(json-c-static PRIVATE /wd4505 /wd4189)
    endif()
endif()
if (WITH_EXAMPLES AND NOT NXP)
        add_subdirectory(third-party/imgui)
endif()