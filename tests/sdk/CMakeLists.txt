cmake_minimum_required(VERSION 3.15)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
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

# Add tests
add_aditof_test(generic-sdk_version_test generic/generic-sdk_version_test.cpp LABELS "unit")
add_aditof_test(system-get_camera_list system/system-get_camera_list.cpp LABELS "integration;hardware")
add_aditof_test(camera-get_frames camera/camera-get_frames.cpp LABELS "integration;hardware")
add_aditof_test(camera-adsd3500_reset camera/camera-adsd3500_reset.cpp LABELS "integration;hardware")
add_aditof_test(camera-parameters camera/camera-parameters.cpp LABELS "integration;hardware")
add_aditof_test(libaditof_sample_test sample/sample_test.cpp LABELS "unit;integration")

# Phase 2: Core Workflow Unit Tests
add_aditof_test(frame-operations_test frame/frame-operations_test.cpp LABELS "unit")

# Phase 3: Status & Type Definitions Tests
add_aditof_test(definitions-test definitions/definitions-test.cpp LABELS "unit")

# Phase 4: Camera API Unit Tests
add_aditof_test(camera-api_test camera/camera-api_test.cpp LABELS "unit;api")

# Phase 5: Security Validation Tests
add_aditof_test(platform-security_test platform/platform-security_test.cpp LABELS "unit;security")

# Copy camera parameters test data if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/camera/data/camera-parameters/)
    file(COPY camera/data/camera-parameters/ DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

# Note: Test labels allow filtering:
#   ctest -L unit           # Run only unit tests
#   ctest -L integration    # Run integration tests
#   ctest -L hardware       # Run tests that require hardware
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/camera/data/camera-parameters/)
    file(COPY camera/data/camera-parameters/ DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()