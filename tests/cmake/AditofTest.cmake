# AditofTest.cmake - Shared test utilities for aditof test suites
#
# Usage:
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AditofTest.cmake)
#   add_aditof_test(test-name source.cpp [source2.cpp ...])

# Helper function to add a test with common configuration
# Creates a GTest executable linked against gtest, gtest_main, and aditof
#
# Parameters:
#   TEST_NAME - Name of the test executable target
#   ARGN      - List of source files for the test
function(add_aditof_test TEST_NAME)
    # Create executable from all source files passed after TEST_NAME
    add_executable(${TEST_NAME} ${ARGN})
    
    # Link common dependencies
    target_link_libraries(${TEST_NAME}
        PRIVATE
            gtest
            gtest_main
            aditof
            json-c
            aditof_test_utils
    )
    
    # Include directories - assumes test is in tests/sdk/<category>/
    target_include_directories(${TEST_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../include
            ${CMAKE_BINARY_DIR}/dependencies/third-party/json-c
    )
    
    # Register with CTest for test discovery
    gtest_discover_tests(${TEST_NAME})
endfunction()
