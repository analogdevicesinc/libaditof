cmake_minimum_required(VERSION 3.15)

# Include GoogleTest module
include(GoogleTest)

# Include shared test utilities
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/AditofTest.cmake)

# To add more tests, just add one line per test:
add_aditof_test(crosby_mode_get_frames crosby_mode_get_frames.cpp)
