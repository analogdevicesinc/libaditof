cmake_minimum_required(VERSION 3.0)
project(libaditof_project)

####################### Disable In-source builds ##############################
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
    message(FATAL_ERROR "In-source builds are not allowed. \
    Consider making a separate build folder and run cmake \
    from there using this command:
    cmake ${CMAKE_SOURCE_DIR}")
endif()

############################### Version #######################################
set(ADITOF_VERSION_MAJOR 6)
set(ADITOF_VERSION_MINOR 0)
set(ADITOF_VERSION_PATCH 0-BETA)

set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

############################### Options #######################################
option(WITH_DOC "Build documentation?" OFF)
option(WITH_NETWORK "Build network interface?" ON)
option(WITH_OFFLINE "Build offline interface?" OFF)
option(WITH_GLOG_DEPENDENCY "Build with GLOG dependency to be used for logging?" OFF)
option(CI_BUILD "Build for CI" OFF)
option(WITH_SUBMODULES "Use submodules to build the library" ON)
option(ENABLE_PCM "Enable the streaming of PCM frames" OFF)

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

if (WITH_NETWORK)
        add_definitions(-DHAS_NETWORK)
endif()

set(RESOURCES_DIR "${CMAKE_BINARY_DIR}/resources")
make_directory(${RESOURCES_DIR})

if (WITH_OFFLINE)
        add_definitions(-DHAS_OFFLINE)

        set(RESOURCES_OFFLINE_DIR "${RESOURCES_DIR}/offline")
        make_directory(${RESOURCES_OFFLINE_DIR})
        
        set(MODE_FILE "lr-native.bin" "sr-native.bin" "lr-qnative.bin" "sr-qnative.bin" "lr-mixed.bin" "sr-mixed.bin")
        message("Downloading raw frames into ${RESOURCES_OFFLINE_DIR}")
        foreach(MODE IN LISTS MODE_FILE)
                set(MODE_URL "swdownloads.analog.com/cse/aditof/resources/itof/frames/${MODE}")
                set(MODE_PATH "${RESOURCES_OFFLINE_DIR}/${MODE}")

                if (NOT EXISTS ${MODE_PATH})
                        file(DOWNLOAD "${MODE_URL}" "${MODE_PATH}")
                endif()
        endforeach()

        if (ON_TARGET)
                make_directory("${RESOURCES_OFFLINE_DIR}/adsd3500_raw")

                message("Downloading raw frames into ${RESOURCES_OFFLINE_DIR}/adsd3500_raw")
                foreach(MODE IN LISTS MODE_FILE)
                set(MODE_URL "swdownloads.analog.com/cse/aditof/resources/itof/offline/adsd3500_raw/${MODE}")
                set(MODE_PATH "${RESOURCES_OFFLINE_DIR}/adsd3500_raw/${MODE}")

                if (NOT EXISTS ${MODE_PATH})
                        file(DOWNLOAD "${MODE_URL}" "${MODE_PATH}")
                endif()
        endforeach()
        endif()
endif()

if(WITH_GLOG_DEPENDENCY)
        add_definitions(-DUSE_GLOG)
        if(WITH_SUBMODULES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/glog/.git" )
                # Temporarily store the current compile options
                set(old_cxx_flags "${CMAKE_CXX_FLAGS}")
                set(old_c_flags "${CMAKE_C_FLAGS}")
                # Disable warnings for this subdirectory
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                set(WITH_GFLAGS OFF CACHE BOOL "Disable gflags")
                set(BUILD_TESTING OFF CACHE BOOL "Disable test builds for glog lib")
                set(POSITION_INDEPENDENT_CODE 1)
                # Add the subdirectory
                add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/glog)
                unset(WITH_GFLAGS)
                unset(POSITION_INDEPENDENT_CODE)
                # Restore the old compile options
                set(CMAKE_CXX_FLAGS "${old_cxx_flags}")
                set(CMAKE_C_FLAGS "${old_c_flags}")
        endif()
endif()

if(WITH_SUBMODULES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libwebsockets/.git" )
        # Temporarily store the current compile options
        set(old_cxx_flags "${CMAKE_CXX_FLAGS}")
        set(old_c_flags "${CMAKE_C_FLAGS}")
        # Disable warnings for this subdirectory
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
        set(LWS_WITH_SSL 0 CACHE BOOL "Disable SSL support in libwebsockets")
        set(LWS_STATIC_PIC 1 CACHE BOOL "Enable static position independent code in libwebsockets")
        set(POSITION_INDEPENDENT_CODE 1)
        # Add the subdirectory
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libwebsockets)
        unset(LWS_WITH_SSL)
        unset(LWS_STATIC_PIC)
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
        if(WITH_SUBMODULES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/protobuf/.git" )
                # Temporarily store the current compile options
                set(old_cxx_flags "${CMAKE_CXX_FLAGS}")
                set(old_c_flags "${CMAKE_C_FLAGS}")
                # Disable warnings for this subdirectory
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                set(protobuf_MSVC_STATIC_RUNTIME 0 CACHE BOOL "MSVC static tuntime disabled")
                set(protobuf_BUILD_TESTS 0 CACHE BOOL "Disable tests for Protobuf")
                set(POSITION_INDEPENDENT_CODE 1)
                # Add the subdirectory
                add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/protobuf/cmake)
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
                set(LIBTOFI_LIBDIR_PATH "${CMAKE_SOURCE_DIR}/../libs")
        endif()
endif()

set(CONFIG_DIR_NAME "config")
add_definitions(-DCONFIG_DIR_NAME="${CONFIG_DIR_NAME}")
add_definitions(-DRESOURCES="${RESOURCES_DIR}")

add_subdirectory(dependencies)
add_subdirectory(sdk)

if (WITH_DOC)
        add_subdirectory(doc)
endif()

############################### Installer #######################################
configure_file(cmake/aditof-setup.iss.cmakein ${CMAKE_CURRENT_BINARY_DIR}/aditof-setup.iss @ONLY)

############################### Version #######################################
configure_file(cmake/version.h.cmakein ${CMAKE_SOURCE_DIR}/sdk/include/aditof/version.h @ONLY)
