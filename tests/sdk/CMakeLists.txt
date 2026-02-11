cmake_minimum_required(VERSION 3.15)
# Set absolute path for test binaries output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests/sdk/bin)
# Include GoogleTest module
include(GoogleTest)

# Build the test utilities library
add_library(aditof_test_utils STATIC
    src/aditof_test_utils.cpp
)

target_include_directories(aditof_test_utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_BINARY_DIR}/dependencies/third-party/json-c
)

target_link_libraries(aditof_test_utils
    PUBLIC
        gtest
        json-c
)

# Include shared test utilities
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AditofTest.cmake)

# Generic tests (Unit Tests & API Tests)
add_aditof_test(generic-sdk_version_test generic/generic-sdk_version_test.cpp)
add_aditof_test(generic-frame_unit_test generic/generic-frame_unit_test.cpp)
add_aditof_test(generic-frame_api_test generic/generic-frame_api_test.cpp)
add_aditof_test(generic-frame_boundary_test generic/generic-frame_boundary_test.cpp)
add_aditof_test(generic-status_api_test generic/generic-status_api_test.cpp)
add_aditof_test(generic-utils_api_test generic/generic-utils_api_test.cpp)
add_aditof_test(generic-sdk_source_integration_test generic/generic-sdk_source_integration_test.cpp)

# System tests (Integration Tests)
add_aditof_test(system-get_camera_list system/system-get_camera_list.cpp)
add_aditof_test(system-integration_test system/system-integration_test.cpp)

# System tests - Conditional (add only if files exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/system/system-connection_test.cpp)
    add_aditof_test(system-connection_test system/system-connection_test.cpp)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/system/system-lifecycle_test.cpp)
    add_aditof_test(system-lifecycle_test system/system-lifecycle_test.cpp)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/system/system-error_handling_test.cpp)
    add_aditof_test(system-error_handling_test system/system-error_handling_test.cpp)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/system/system-camera_operations_test.cpp)
    add_aditof_test(system-camera_operations_test system/system-camera_operations_test.cpp)
endif()

# End-to-End System Tests (Full SDK Workflow Tests)
add_aditof_test(system-e2e_workflow_test system/system-e2e_workflow_test.cpp)
add_aditof_test(system-frame_data_pipeline_test system/system-frame_data_pipeline_test.cpp)
add_aditof_test(system-adsd3500_integration_test system/system-adsd3500_integration_test.cpp)
add_aditof_test(system-frame_handler_test system/system-frame_handler_test.cpp)
add_aditof_test(system-connection_types_test system/system-connection_types_test.cpp)

# Camera tests (Functional Tests)
add_aditof_test(camera-get_frames camera/camera-get_frames.cpp)
add_aditof_test(camera-adsd3500_reset camera/camera-adsd3500_reset.cpp)
add_aditof_test(camera-adsd3500_reset_enhanced camera/camera-adsd3500_reset_enhanced.cpp)
add_aditof_test(camera-functional_test camera/camera-functional_test.cpp)

# Optional tests - only add if files exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/sample/sample_test.cpp)
    add_aditof_test(libaditof_sample_test sample/sample_test.cpp)
endif()

# Camera parameters test - only add if file exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/camera/camera-parameters.cpp)
    add_aditof_test(camera-parameters camera/camera-parameters.cpp)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/camera/data/camera-parameters/)
    file(COPY camera/data/camera-parameters/ DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()