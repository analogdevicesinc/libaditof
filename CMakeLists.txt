cmake_minimum_required(VERSION 3.0)
cmake_policy(VERSION 3.12)
cmake_policy(SET CMP0054 NEW)
cmake_policy(SET CMP0075 NEW)
project(libaditof_project)

####################### Disable In-source builds ##############################
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
    message(FATAL_ERROR "In-source builds are not allowed. \
    Consider making a separate build folder and run cmake \
    from there using this command:
    cmake ${CMAKE_SOURCE_DIR}")
endif()

############################### Version #######################################
set(ADITOF_VERSION_MAJOR 7)
set(ADITOF_VERSION_MINOR 0)
set(ADITOF_VERSION_PATCH 0)
set(ADITOF_VERSION_QUALITY a-1)

set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

############################### Options #######################################
option(WITH_DOC "Build documentation?" OFF)
option(WITH_NETWORK "Build network interface?" ON)
option(WITH_PYTHON "Build python bindings?" ON)
option(WITH_OFFLINE "Build offline interface?" ON)
option(CI_BUILD "Build for CI" OFF)
option(WITH_SUBMODULES "Use submodules to build the library" ON)
option(ENABLE_PCM "Enable the streaming of PCM frames" OFF)
option(RECV_ASYNC "Receive the frames Asynchronously" ON)
option(BUILD_TESTING "Build tests?" OFF)
set(SOM_TYPE "Dual" CACHE STRING "Which SoM SDK is building for?")

######################### Target based Options ################################
option(NXP "Set to ON when building on NXP" OFF)
option(NVIDIA "Set to ON when building on NVIDIA" OFF)
option(WITH_PROTOBUF_DEPENDENCY "Build with PROTOBUF dependency?" ON)
option(USE_DEPTH_COMPUTE_OPENSOURCE "Use an open source implementation?" OFF)

################################## Git ########################################
include(FindGit OPTIONAL)
if (GIT_FOUND)
        execute_process(
                COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                OUTPUT_VARIABLE ADITOFSDK_GIT_REPO
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
        )

        if ("${ADITOFSDK_GIT_REPO}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
                execute_process(
                        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        OUTPUT_VARIABLE ADITOFSDK_GIT_COMMIT
                        OUTPUT_STRIP_TRAILING_WHITESPACE
                )
                execute_process(
                        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        OUTPUT_VARIABLE ADITOFSDK_GIT_BRANCH
                        OUTPUT_STRIP_TRAILING_WHITESPACE
                )
        endif()

        # Updating submodules if they exist
        if (WITH_SUBMODULES)
                add_definitions(-DWITH_SUBMODULES)
                message("Updating submodules")
                execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE WITH_SUBMODULES_RESULT)

                if(NOT WITH_SUBMODULES_RESULT EQUAL "0")
                        message(FATAL_ERROR "Git submodules failed to initialize, check submodules")
                endif()
        endif()

endif()

add_definitions(-DADITOFSDK_GIT_COMMIT="${ADITOFSDK_GIT_COMMIT}")
add_definitions(-DADITOFSDK_GIT_BRANCH="${ADITOFSDK_GIT_BRANCH}")

############################## Rest of cmake ##################################

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Check if CMAKE_BUILD_TYPE has been selected, default: Release
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Check whether we configure things for remote or we're on target
set(SELECTED_PLATFORMS 0)
set(ALL_PLATFORMS NXP;NVIDIA)
foreach(platform IN LISTS ALL_PLATFORMS)
    if(${platform})
        math(EXPR SELECTED_PLATFORMS "${SELECTED_PLATFORMS}+1")
    endif()
endforeach()

set(ON_TARGET FALSE)
if(${SELECTED_PLATFORMS} GREATER 1)
        message(FATAL_ERROR "More than one platform was selected. \nPlease select a single platform. CMake will exit.")
elseif(${SELECTED_PLATFORMS} EQUAL 1)
    add_definitions(-DTARGET)
    set(ON_TARGET TRUE)
endif()

if(NOT ON_TARGET AND USE_DEPTH_COMPUTE_OPENSOURCE)
        message (STATUS "USE_DEPTH_COMPUTE_OPENSOURCE is not used on host")
endif()

if(CI_BUILD)
    set(USE_DEPTH_COMPUTE_OPENSOURCE ON)
endif()

if(ENABLE_PCM)
    add_definitions(-DENABLE_PCM)
endif()

if (RECV_ASYNC)
        message("Building Host App to receive frames Asynchronously from the Target.")
        add_definitions(-DRECV_ASYNC)
    else()
        message("Building Host App to receive frames Synchronously from the Target.")
endif()

if (WITH_NETWORK)
        add_definitions(-DHAS_NETWORK)
endif()

if(SOM_TYPE STREQUAL "Dual")
        message("Building for Dual pulsatrix")
        add_definitions(-DDUAL)
else()
        add_definitions(-DSINGLE)
endif()

set(RESOURCES_DIR "${CMAKE_BINARY_DIR}/resources")
make_directory(${RESOURCES_DIR})

if (WITH_OFFLINE)
        add_definitions(-DHAS_OFFLINE)
endif()


if(WITH_SUBMODULES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/third-party/libzmq/.git")
                # Temporarily store the current compile options
                set(old_cxx_flags "${CMAKE_CXX_FLAGS}")
                set(old_c_flags "${CMAKE_C_FLAGS}")
                # Disable warnings for this subdirectory
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                set(BUILD_SHARED OFF CACHE BOOL "Disable shared libs")
                set(BUILD_STATIC ON CACHE BOOL "Build Static library")
                set(BUILD_TESTS OFF CACHE BOOL "Disable building tests for libzmq")
                set(WITH_LIBSODIUM OFF CACHE BOOL "Disable libsodium for libzmq")
                set(POSITION_INDEPENDENT_CODE 1)
                add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/third-party/libzmq)
                unset(POSITION_INDEPENDENT_CODE)
                # Restore the old compile options
                set(CMAKE_CXX_FLAGS "${old_cxx_flags}")
                set(CMAKE_C_FLAGS "${old_c_flags}")
endif()

if(NOT WITH_PROTOBUF_DEPENDENCY)
        if(NOT ON_TARGET)
                message(FATAL_ERROR "SDK can be built without protobuf only on target builds!")
        endif()
else()
        add_definitions(-DUSE_PROTOBUF)
        if(WITH_NETWORK AND WITH_SUBMODULES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/third-party/protobuf/.git" )
                # Temporarily store the current compile options
                set(old_cxx_flags "${CMAKE_CXX_FLAGS}")
                set(old_c_flags "${CMAKE_C_FLAGS}")
                # Disable warnings for this subdirectory
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                                set(BUILD_SHARED_LIBS OFF CACHE BOOL "Disble shared libs")
                set(protobuf_MSVC_STATIC_RUNTIME 0 CACHE BOOL "MSVC static tuntime disabled")
                set(protobuf_BUILD_TESTS 0 CACHE BOOL "Disable tests for Protobuf")
                set(POSITION_INDEPENDENT_CODE 1)
                # Add the subdirectory
                add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/third-party/protobuf/cmake)
                unset(protobuf_BUILD_TESTS)
                unset(POSITION_INDEPENDENT_CODE)
                # Restore the old compile options
                set(CMAKE_CXX_FLAGS "${old_cxx_flags}")
                set(CMAKE_C_FLAGS "${old_c_flags}")
        endif()
endif()

if(${ON_TARGET})
        if(USE_DEPTH_COMPUTE_OPENSOURCE)
                set(LIBTOFI_LIBDIR_PATH "${CMAKE_BINARY_DIR}/sdk/common/adi/depth-compute-opensource")
        else()
                if (NOT DEFINED LIBTOFI_LIBDIR_PATH)
                        set(LIBTOFI_LIBDIR_PATH "${CMAKE_SOURCE_DIR}/../libs")
                endif()
        endif()
endif()

set(CONFIG_DIR_NAME "config")
add_definitions(-DCONFIG_DIR_NAME="${CONFIG_DIR_NAME}")
add_definitions(-DRESOURCES="${RESOURCES_DIR}")

add_subdirectory(dependencies)
add_subdirectory(sdk)

if (BUILD_TESTING)
    # Configure Google Test
    set(BUILD_GMOCK ON CACHE BOOL "Enable building Google Mock")
    set(INSTALL_GTEST OFF CACHE BOOL "Disable installation of googletest")
    add_subdirectory(dependencies/third-party/gtest)
    add_subdirectory(tests)
    enable_testing()
endif()

if (WITH_PYTHON)
        add_subdirectory(bindings/python)
endif()

if (WITH_DOC)
        add_subdirectory(doc)
endif()

############################### Installer #######################################
configure_file(cmake/aditof-setup.iss.cmakein ${CMAKE_CURRENT_BINARY_DIR}/aditof-setup.iss @ONLY)

############################### Version #######################################
configure_file(cmake/version.h.cmakein ${CMAKE_SOURCE_DIR}/sdk/include/aditof/version.h @ONLY)
