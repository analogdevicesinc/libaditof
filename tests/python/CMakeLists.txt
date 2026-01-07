cmake_minimum_required(VERSION 3.15)

# Find pytest
find_program(PYTEST_EXECUTABLE pytest)

if(PYTEST_EXECUTABLE)
    message(STATUS "pytest found: ${PYTEST_EXECUTABLE}")
    
    # Add pytest test runner
    add_test(
        NAME python_bindings_tests
        COMMAND ${PYTEST_EXECUTABLE} -v --tb=short
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
    # Set environment variable to help find the aditofpython module
    set_tests_properties(python_bindings_tests PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}:${CMAKE_BINARY_DIR}/sdk"
    )
else()
    message(WARNING "pytest not found - Python tests will not be run. Install pytest to enable Python tests: pip install pytest")
endif()
