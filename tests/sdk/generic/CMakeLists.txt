cmake_minimum_required(VERSION 3.15)

# Include GoogleTest module
include(GoogleTest)

# Include shared test utilities
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/AditofTest.cmake)

# Add all test executables - one line per test!
add_aditof_test(generic-sdk_version_test generic-sdk_version_test.cpp)

# To add more tests, just add one line per test:
# add_aditof_test(generic-another_test another_test.cpp)
# add_aditof_test(generic-multi_file_test test1.cpp test2.cpp helper.cpp)
