cmake_minimum_required(VERSION 3.15)

# Find Python3 and pytest
find_package(Python3 COMPONENTS Interpreter)

if(Python3_FOUND)
    # Check if pytest is available
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pytest --version
        RESULT_VARIABLE PYTEST_RESULT
        OUTPUT_QUIET ERROR_QUIET
    )
    
    if(PYTEST_RESULT EQUAL 0)
        message(STATUS "pytest found - Python tests will be enabled")
        
        # Add pytest test runner
        add_test(
            NAME PythonBindingsTests
            COMMAND ${Python3_EXECUTABLE} -m pytest -v --tb=short --color=yes
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
        
        # Set PYTHONPATH to find aditofpython module
        # The module is built at CMAKE_BINARY_DIR/libaditof/bindings/python/
        set_tests_properties(PythonBindingsTests PROPERTIES
            ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/libaditof/bindings/python:$ENV{PYTHONPATH}"
            LABELS "python;unit"
            TIMEOUT 60
        )
        
        message(STATUS "Python tests registered - run with: ctest -L python")
    else()
        message(WARNING "pytest not found - Install with: pip3 install pytest numpy")
    endif()
else()
    message(WARNING "Python3 not found - Python tests will not be available")
endif()
