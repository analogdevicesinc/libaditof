cmake_minimum_required(VERSION 3.0)
project(aditof)

############################### Version #######################################
configure_file(../cmake/version.h.cmakein ${CMAKE_CURRENT_SOURCE_DIR}/include/aditof/version.h @ONLY)

if(UNIX)
    set(CMAKE_POSITION_INDEPENDENT_CODE "1")
endif()

add_definitions(-DSDK_EXPORTS)

# Add general optimization flag
if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-Ofast)
endif()

if(WITH_GLOG_DEPENDENCY)
    find_package(glog 0.3.5 REQUIRED)
    set(GLOG_BIN_DIR "${glog_DIR}/../../../bin")
    set(GLOG_BIN_DIR ${GLOG_BIN_DIR} PARENT_SCOPE)
endif()

if(WITH_PROTOBUF_DEPENDENCY)
    find_package(Protobuf 3.9.0 REQUIRED)
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS src/connections/usb/usb_buffer.proto)
    get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HDRS} DIRECTORY)

    if ( WITH_NETWORK )
        find_package(Libwebsockets REQUIRED)
        set(LWS_CMAKE_DIR ${LWS_CMAKE_DIR} PARENT_SCOPE)

        protobuf_generate_cpp(NET_PROTO_SRCS NET_PROTO_HRDS src/connections/network/buffer.proto)
        get_filename_component(GENERATED_NET_PROTO_FILES_DIR ${NET_PROTO_HRDS} DIRECTORY)
    endif()
endif()

file(GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

file(GLOB HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/aditof/*.h
)

set(CAMERA_DIR "cameras/itof-camera")
set(FRAME_DIR "cameras/itof-frame")
set(COMMON_DIR "common/adi")

if( WIN32 )
        set(OS_SPECIFIC_DIR "connections/usb/windows")
elseif( APPLE )
        set(OS_SPECIFIC_DIR "connections/usb/macos")
elseif ( NXP )
        # Add optimization for specific architecture
        if(CMAKE_BUILD_TYPE STREQUAL "Release")
                add_compile_options(-march=armv8-a+simd)
        endif()
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR "soc/imx")
        add_definitions(-DNXP)
elseif ( NVIDIA )
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR "soc/nvidia")
        add_definitions(-DNVIDIA)
elseif ( UNIX )
        set(OS_SPECIFIC_DIR "connections/usb/linux")
else()
    message(FATAL_ERROR "Platform not supported")
endif()

if (NOT APPLE AND NOT WIN32)
    option(RUN_LDCONFIG "Run ldconfig after install." ON)
endif()

# Platform specific sources and headers
file(GLOB PLATFORM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/adsd3500/crc/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/${TARGET_SPECIFIC_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${FRAME_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_DIR}/tofi/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_DIR}/tofi/*.cpp
)
file(GLOB PLATFORM_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/adsd3500/crc/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/${TARGET_SPECIFIC_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${FRAME_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_DIR}/tofi/*.h
)

if(USE_DEPTH_COMPUTE_OPENSOURCE AND ON_TARGET)
    add_subdirectory(common/adi/depth-compute-opensource)
endif()

file(GLOB USB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/usb/*.cpp
)
file(GLOB USB_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/usb/*.h
)

file(GLOB NETWORK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/network/*.cpp
)
file(GLOB NETWORK_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/network/*.h
)

file(GLOB OFFLINE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/offline/*.cpp
)
file(GLOB OFFLINE_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/offline/*.h
)

# Create target and set properties
add_library(${PROJECT_NAME} SHARED
    ${HEADERS}
    ${SOURCES}
    ${PLATFORM_SOURCES}
    ${PLATFORM_HEADERS}
    $<$<BOOL:${WITH_NETWORK}>:${NETWORK_SOURCES}>
    $<$<BOOL:${WITH_NETWORK}>:${NETWORK_HEADERS}>
    $<$<BOOL:${WITH_NETWORK}>:${NET_PROTO_SRCS}>
    $<$<BOOL:${WITH_NETWORK}>:${NET_PROTO_HDRS}>
    $<$<BOOL:${WITH_OFFLINE}>:${OFFLINE_SOURCES}>
    $<$<BOOL:${WITH_OFFLINE}>:${OFFLINE_HEADERS}>
    $<$<BOOL:${WITH_PROTOBUF_DEPENDENCY}>:${PROTO_SRCS}>
    $<$<BOOL:${WITH_PROTOBUF_DEPENDENCY}>:${PROTO_HDRS}>
    $<$<BOOL:${WITH_PROTOBUF_DEPENDENCY}>:${USB_SOURCES}>
    $<$<BOOL:${WITH_PROTOBUF_DEPENDENCY}>:${USB_HEADERS}>
)

target_link_libraries(${PROJECT_NAME} PRIVATE crc32)
target_link_libraries(${PROJECT_NAME} PRIVATE cjson)

# Add alias to aditof as aditof::aditof
add_library(aditof::aditof ALIAS aditof)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        src/
        src/${OS_SPECIFIC_DIR}/${TARGET_SPECIFIC_DIR}
        src/${CAMERA_DIR}/
        ${COMMON_DIR}/
        ${Protobuf_INCLUDE_DIRS}
        ${GENERATED_NET_PROTO_FILES_DIR}
        ${GENERATED_PROTO_FILES_DIR}
        ${GENERATED_NET_PROTO_FILES_DIR}
        ${LIBWEBSOCKETS_INCLUDE_DIRS}
        ${OPENSSL_INCLUDE_DIRS}
)

set_target_properties( ${PROJECT_NAME} PROPERTIES
                       VERSION ${VERSION}
                       SOVERSION 1.0
                       CXX_STANDARD 11
)

if ( WITH_NETWORK )
    target_link_libraries( ${PROJECT_NAME}
        PRIVATE
            websockets_shared
    )
endif()

if (WITH_PROTOBUF_DEPENDENCY)
    target_link_libraries( ${PROJECT_NAME}
        PRIVATE
            ${Protobuf_LIBRARIES}
    )
endif()

if (WITH_GLOG_DEPENDENCY)
    target_link_libraries( ${PROJECT_NAME}
    PUBLIC
        glog::glog
    )
endif()

#LIBTOFI_LIBDIR_PATH - path to libtofi_config.so and libtofi_compute.so folder	
set(LIBTOFI_COMPUTE_CLOSED_SOURCE_PATH "${LIBTOFI_LIBDIR_PATH}/libtofi_compute.so")
set(LIBTOFI_CONFIG_CLOSED_SOURCE_PATH "${LIBTOFI_LIBDIR_PATH}/libtofi_config.so")

if ( ON_TARGET AND UNIX )
	if (NOT USE_DEPTH_COMPUTE_OPENSOURCE)
                add_library(libtofi_compute SHARED IMPORTED)
                set_target_properties(libtofi_compute PROPERTIES
                IMPORTED_LOCATION "${LIBTOFI_COMPUTE_CLOSED_SOURCE_PATH}"
                IMPORTED_NO_SONAME 1
                )

                add_library(libtofi_config SHARED IMPORTED)
                set_target_properties(libtofi_config PROPERTIES
                        IMPORTED_LOCATION "${LIBTOFI_CONFIG_CLOSED_SOURCE_PATH}"
                        IMPORTED_NO_SONAME 1
                        )

                target_link_libraries(${PROJECT_NAME} PRIVATE libtofi_compute)
                target_link_libraries(${PROJECT_NAME} PRIVATE libtofi_config)
    else()
        target_link_libraries(${PROJECT_NAME} PRIVATE tofi_compute)
        target_link_libraries(${PROJECT_NAME} PRIVATE tofi_config)
    endif()
endif()	


if (CMAKE_COMPILER_IS_GNUCC)
    target_compile_options(${PROJECT_NAME} PUBLIC
            -Wall
            -Wno-unknown-pragmas
            -Werror=return-type
            -Werror=uninitialized
            -Werror=init-self
    )
endif()

if( APPLE )
    find_library( LIBUVC_LIB libuvc.dylib /usr/local/lib/ )
    if( NOT LIBUVC_LIB)
        MESSAGE( SEND_ERROR "Cannot find the libuvc library")
    endif( NOT LIBUVC_LIB)
    target_link_libraries( ${PROJECT_NAME} PRIVATE ${LIBUVC_LIB})
endif( APPLE )

# includes install paths
include(GNUInstallDirs)

# install path for the aditof-config.cmake and aditof-config-version.cmake
set(INSTALL_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/aditof)

if (ON_TARGET) # ON TARGET
    if(USE_DEPTH_COMPUTE_OPENSOURCE) # OPEN SOURCE LIBRARIES
        install(TARGETS aditof tofi_compute tofi_config
            EXPORT aditof-targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )
    else() # CLOSED SOURCE LIBRARIES
        install(TARGETS aditof 
            EXPORT aditof-targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )
        install(FILES
            ${LIBTOFI_COMPUTE_CLOSED_SOURCE_PATH}
            ${LIBTOFI_CONFIG_CLOSED_SOURCE_PATH}
            DESTINATION
            ${CMAKE_INSTALL_LIBDIR}
        )
    endif()
else() # NOT ON TARGET, NO LIBRARIES
    install(TARGETS aditof
        EXPORT aditof-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

install(EXPORT aditof-targets
    FILE
        aditof-targets.cmake
    NAMESPACE
        aditof::
    DESTINATION
        ${INSTALL_CONFIG_DIR}
)

# install the include folder
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# include helper functions to write the config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config-version.cmake
    VERSION ${VERSION}
    COMPATIBILITY AnyNewerVersion
)

set(CONFIG_INCLUDE_DIRS "${CMAKE_PREFIX_PATH}/${CMAKE_INSTALL_INCLUDEDIR}")

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/aditof-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

# install the files in order to be used by other projects
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config-version.cmake
    DESTINATION ${INSTALL_CONFIG_DIR}
)
