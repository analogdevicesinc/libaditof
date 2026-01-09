cmake_minimum_required(VERSION 3.15)

# Add test executables
add_executable(libaditof_sample_test
    sample_test.cpp
)

# Link against Google Test and the aditof library
target_link_libraries(libaditof_sample_test
    PRIVATE
        gtest
        gtest_main
        aditof
)

# Include directories
target_include_directories(libaditof_sample_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Add tests to CTest
include(GoogleTest)
gtest_discover_tests(libaditof_sample_test)