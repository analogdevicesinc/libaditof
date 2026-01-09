cmake_minimum_required(VERSION 3.15)

# Include GoogleTest module
include(GoogleTest)

# Include shared test utilities
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/AditofTest.cmake)

# To add more tests, just add one line per test:
add_aditof_test(camera-get_frames_test camera-get_frames_test.cpp)
add_aditof_test(camera-adsd3500Reset camera-adsd3500Reset.cpp)
