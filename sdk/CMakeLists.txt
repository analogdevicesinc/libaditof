cmake_minimum_required(VERSION 3.15)
project(aditof)

########################### Version #######################################
configure_file(../cmake/version.h.cmakein ${CMAKE_CURRENT_SOURCE_DIR}/include/aditof/version.h @ONLY)

if(UNIX)
    set(CMAKE_POSITION_INDEPENDENT_CODE "1")
endif()

add_definitions(-DSDK_EXPORTS)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT (WIN32 OR ON_TARGET))
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0.0")
        message(STATUS "Applying flags for GCC/G++ version 13.x or higher")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include cstdint")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include stdint.h")
    else()
        message(STATUS "Skipping flags for GCC/G++ version ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
endif()


# Add general optimization flag
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-Ofast)
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

if ( NXP )
    # Add optimization for specific architecture
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-march=armv8-a+simd)
    endif()
    set(OS_SPECIFIC_DIR "connections/target")
    set(PLATFORM_DIR "platform/imx")
    add_definitions(-DNXP)
elseif ( NVIDIA )
    set(OS_SPECIFIC_DIR "connections/target")
    set(PLATFORM_DIR "platform/nvidia")
    add_definitions(-DNVIDIA)
elseif ( RPI )
    set(OS_SPECIFIC_DIR "connections/target")
    set(PLATFORM_DIR "platform/raspberrypi")
    add_definitions(-DRPI)
else()
    message(FATAL_ERROR "Platform not supported. Use -DNVIDIA=ON, -DNXP=ON, or -DRPI=ON")
endif()

if (NOT APPLE AND NOT WIN32)
    option(RUN_LDCONFIG "Run ldconfig after install." ON)
endif()

# Platform layer sources and headers (common)
set(PLATFORM_COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/sensor_enumerator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/platform_factory.cpp
)
set(PLATFORM_COMMON_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/sensor_enumerator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/platform_factory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/platform_interface.h
)

# Platform-specific implementation sources
file(GLOB PLATFORM_IMPL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${PLATFORM_DIR}/*.cpp
)
file(GLOB PLATFORM_IMPL_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${PLATFORM_DIR}/*.h
)

# Platform specific sources and headers
file(GLOB PLATFORM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/sensor-tables/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/adsd3500/crc/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${FRAME_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_DIR}/tofi/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_DIR}/tofi/*.cpp
)
file(GLOB PLATFORM_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/sensor-tables/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/adsd3500/crc/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${FRAME_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_DIR}/tofi/*.h
)

if(USE_DEPTH_COMPUTE_OPENSOURCE AND ON_TARGET)
    add_subdirectory(common/adi/depth-compute-opensource)
endif()

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
    ${PLATFORM_COMMON_SOURCES}
    ${PLATFORM_COMMON_HEADERS}
    ${PLATFORM_IMPL_SOURCES}
    ${PLATFORM_IMPL_HEADERS}
    ${PLATFORM_SOURCES}
    ${PLATFORM_HEADERS}
    $<$<BOOL:${WITH_NETWORK}>:${NETWORK_SOURCES}>
    $<$<BOOL:${WITH_NETWORK}>:${NETWORK_HEADERS}>
    $<$<BOOL:${WITH_OFFLINE}>:${OFFLINE_SOURCES}>
    $<$<BOOL:${WITH_OFFLINE}>:${OFFLINE_HEADERS}>
)

target_link_libraries(${PROJECT_NAME} PRIVATE crc32)
target_link_libraries(${PROJECT_NAME} PRIVATE json-c)
# json-c may need libbsd on Linux/Unix systems
if(UNIX AND NOT APPLE)
    find_library(BSD_LIBRARY bsd)
    if(BSD_LIBRARY)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${BSD_LIBRARY})
    endif()
endif()

# Add json-c include directories (both source and build for generated headers)
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/third-party/json-c
    ${CMAKE_BINARY_DIR}/libaditof/dependencies/third-party/json-c
)

if(WITH_NETWORK AND WITH_PROTOBUF_DEPENDENCY)
    # Check if submodule exists, and build the library
    if(NOT WITH_SUBMODULES AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/third-party/protobuf/.git")
        find_package(Protobuf 3.9.0 REQUIRED)
    else()
    target_link_libraries(${PROJECT_NAME} PUBLIC libprotobuf)

    endif()
    target_sources(${PROJECT_NAME} PRIVATE ${PROTO_HDRS} ${PROTO_SRCS})
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    if ( WITH_NETWORK )
        # Check if submodule exists, and build the library
        # Include the cppzmq to use in c++
		set(CPPZMQ_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/third-party/cppzmq")
		
		# Check if submodule exists, and build the library
		if(NOT WITH_SUBMODULES AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/third-party/libzmq/.git")
			find_package(ZeroMQ REQUIRED)
		else()
			include_directories(${CPPZMQ_INSTALL_DIR})
			target_link_libraries(${PROJECT_NAME} PUBLIC libzmq-static)
		endif()
        set(NET_PROTO_HRDS "${CMAKE_CURRENT_SOURCE_DIR}/buffer.pb.h")
        set(NET_PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/buffer.pb.cc")
		add_custom_command(OUTPUT ${NET_PROTO_HRDS} ${NET_PROTO_SRCS}
                            COMMAND protoc buffer.proto -I ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/network --cpp_out=.
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        target_sources(${PROJECT_NAME} PRIVATE ${NET_PROTO_HRDS} ${NET_PROTO_SRCS})
        target_include_directories(${PROJECT_NAME} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}
        )
    endif()
endif()

# Add alias to aditof as aditof::aditof
add_library(aditof::aditof ALIAS aditof)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        src/
        src/platform/
        src/${CAMERA_DIR}/
        ${COMMON_DIR}/
        ${Protobuf_INCLUDE_DIRS}
        ${GENERATED_NET_PROTO_FILES_DIR}
        ${GENERATED_PROTO_FILES_DIR}
        ${GENERATED_NET_PROTO_FILES_DIR}
        ${OPENSSL_INCLUDE_DIRS}
)


if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT (WIN32 OR ON_TARGET))
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0.0")
        message(STATUS "Applying flags for GCC/G++ version 13.x or higher")
        set_target_properties( ${PROJECT_NAME} PROPERTIES
                       VERSION ${VERSION}
                       SOVERSION 1.0
                       CXX_STANDARD 14
                       ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
                       LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
                       RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        )
    endif()
else()
    set_target_properties( ${PROJECT_NAME} PROPERTIES
                           VERSION ${VERSION}
                           SOVERSION 1.0
                           CXX_STANDARD 14
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


if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME} PUBLIC
            -Wall
            -Wno-unknown-pragmas
            -Werror=return-type
            -Werror=uninitialized
            -Werror=init-self
    )
endif()

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
