# AditofTest.cmake - Shared test utilities for aditof test suites
#
# Usage:
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AditofTest.cmake)
#   add_aditof_test(test-name source.cpp [source2.cpp ...] [LABELS label1 label2 ...])

# Helper function to add a test with common configuration
# Creates a GTest executable linked against gtest, gtest_main, and aditof
#
# Parameters:
#   TEST_NAME - Name of the test executable target
#   ARGN      - List of source files and optional LABELS argument
function(add_aditof_test TEST_NAME)
    # Parse arguments to separate sources from labels
    cmake_parse_arguments(TEST "" "" "LABELS" ${ARGN})
    
    # Create executable from source files
    add_executable(${TEST_NAME} ${TEST_UNPARSED_ARGUMENTS})
    
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
    if(TEST_LABELS)
        # Pass LABELS as a property to all discovered tests
        gtest_discover_tests(${TEST_NAME}
            PROPERTIES
                LABELS "${TEST_LABELS}"
        )
    else()
        gtest_discover_tests(${TEST_NAME})
    endif()
endfunction()
