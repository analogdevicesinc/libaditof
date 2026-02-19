cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0075 NEW)
project(dependecies)

add_subdirectory(third-party/json-c)

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
    # Mark json-c includes as SYSTEM to suppress warnings in consuming targets
    get_target_property(JSON_C_INCLUDE_DIRS json-c-static INTERFACE_INCLUDE_DIRECTORIES)
    if(JSON_C_INCLUDE_DIRS)
        set_target_properties(json-c-static PROPERTIES 
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${JSON_C_INCLUDE_DIRS}")
    endif()
endif()

if(TARGET json-c)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(json-c PRIVATE 
            -Wno-unused-function 
            -Wno-unused-variable 
            -Wno-unused-but-set-variable)
    elseif(MSVC)
        target_compile_options(json-c PRIVATE /wd4505 /wd4189)
    endif()
    # Mark json-c includes as SYSTEM to suppress warnings in consuming targets
    get_target_property(JSON_C_INCLUDE_DIRS json-c INTERFACE_INCLUDE_DIRECTORIES)
    if(JSON_C_INCLUDE_DIRS)
        set_target_properties(json-c PROPERTIES 
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${JSON_C_INCLUDE_DIRS}")
    endif()
endif()

add_subdirectory(third-party/crc32)
