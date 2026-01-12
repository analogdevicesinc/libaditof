cmake_minimum_required(VERSION 3.15)

# Include GoogleTest module
include(GoogleTest)

# Include shared test utilities
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/AditofTest.cmake)

# Add all test executables - one line per test!
add_aditof_test(system-get_camera_list system-get_camera_list.cpp)