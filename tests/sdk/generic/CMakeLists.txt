cmake_minimum_required(VERSION 3.15)

# Add test executables
add_executable(generic-sdk_version_test
    generic-sdk_version_test.cpp
)

# Link against Google Test and the aditof library
target_link_libraries(generic-sdk_version_test
    PRIVATE
        gtest
        gtest_main
        aditof
)

# Include directories
target_include_directories(generic-sdk_version_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Add tests to CTest
include(GoogleTest)
gtest_discover_tests(generic-sdk_version_test)
