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
        
        # Discover individual Python tests at configure time
        # Note: Discovery may fail if aditofpython not built yet, so we fallback to hardcoded list
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "
import sys
sys.path.insert(0, '${CMAKE_BINARY_DIR}/libaditof/bindings/python')
try:
    import aditofpython
    # If module loads, run pytest discovery
    import subprocess
    result = subprocess.run(
        ['${Python3_EXECUTABLE}', '-m', 'pytest', '--collect-only', '-q', 'test_aditof_bindings.py'],
        capture_output=True, text=True, cwd='${CMAKE_CURRENT_SOURCE_DIR}'
    )
    print(result.stdout)
except ImportError:
    # Module not built yet, output placeholder
    print('MODULE_NOT_BUILT')
"
            OUTPUT_VARIABLE PYTEST_OUTPUT
            ERROR_QUIET
        )
        
        # Parse test names from pytest output
        string(REPLACE "\n" ";" TEST_LINES "${PYTEST_OUTPUT}")
        set(TEST_COUNT 0)
        set(PYTHON_TESTS "")
        
        foreach(LINE ${TEST_LINES})
            # Match lines like: test_aditof_bindings.py::TestClass::test_name
            if(LINE MATCHES "test_aditof_bindings.py::(.*)::(.*)")
                set(TEST_CLASS ${CMAKE_MATCH_1})
                set(TEST_NAME ${CMAKE_MATCH_2})
                list(APPEND PYTHON_TESTS "${TEST_CLASS}::${TEST_NAME}")
                math(EXPR TEST_COUNT "${TEST_COUNT} + 1")
            endif()
        endforeach()
        
        # If discovery failed or module not built, use hardcoded test list
        if(TEST_COUNT EQUAL 0 OR PYTEST_OUTPUT MATCHES "MODULE_NOT_BUILT")
            message(STATUS "Python module not built yet - using hardcoded test list")
            set(PYTHON_TESTS
                "TestVersionAPI::test_api_version_returns_string"
                "TestVersionAPI::test_branch_version_returns_string"
                "TestVersionAPI::test_commit_version_returns_string"
                "TestSystemAPI::test_system_instantiation"
                "TestSystemAPI::test_get_camera_list_without_ip"
                "TestSystemAPI::test_camera_list_returns_list"
                "TestSystemAPI::test_get_camera_list_with_ip"
                "TestFrameAPI::test_frame_instantiation"
                "TestFrameAPI::test_frame_get_details"
                "TestFrameAPI::test_frame_data_details_access"
                "TestCameraAPI::test_camera_get_details"
                "TestCameraAPI::test_camera_get_available_modes"
                "TestCameraAPI::test_camera_set_mode"
                "TestCameraAPI::test_camera_start_stop"
                "TestCameraAPI::test_camera_request_frame"
                "TestFrameDataAccess::test_frame_data_details_width_height"
                "TestFrameDataAccess::test_frame_details_structure"
                "TestFrameDataAccess::test_metadata_structure"
                "TestStatusEnum::test_status_ok_value"
                "TestStatusEnum::test_status_generic_error_exists"
                "TestStatusEnum::test_status_invalid_argument_exists"
                "TestCameraDetails::test_camera_details_has_required_fields"
                "TestSensorInterface::test_camera_get_sensor"
                "TestSensorInterface::test_sensor_callback_registration"
                "TestNumpyIntegration::test_frame_data_to_numpy_array"
                "TestNumpyIntegration::test_numpy_array_creation"
                "TestIntegrationScenarios::test_system_initialization_workflow"
                "TestIntegrationScenarios::test_camera_initialization_workflow"
            )
            # TestFrameTypeStrings has 5 parameterized tests (depth, ir, ab, xyz, conf)
            # Register each parameter variant separately
            list(APPEND PYTHON_TESTS
                "TestFrameTypeStrings::test_frame_type_string_validity[depth]"
                "TestFrameTypeStrings::test_frame_type_string_validity[ir]"
                "TestFrameTypeStrings::test_frame_type_string_validity[ab]"
                "TestFrameTypeStrings::test_frame_type_string_validity[xyz]"
                "TestFrameTypeStrings::test_frame_type_string_validity[conf]"
            )
            list(LENGTH PYTHON_TESTS TEST_COUNT)
        endif()
        
        # Register each Python test individually
        foreach(FULL_TEST ${PYTHON_TESTS})
            string(REPLACE "::" "." TEST_DISPLAY_NAME "${FULL_TEST}")
            set(TEST_NAME "Python.${TEST_DISPLAY_NAME}")
            
            add_test(
                NAME ${TEST_NAME}
                COMMAND ${Python3_EXECUTABLE} -m pytest -v --tb=short 
                        "test_aditof_bindings.py::${FULL_TEST}"
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            )
            
            # Set PYTHONPATH for each test
            set_tests_properties(${TEST_NAME} PROPERTIES
                ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/libaditof/bindings/python:$ENV{PYTHONPATH}"
                LABELS "python;unit"
                TIMEOUT 30
            )
        endforeach()
        
        message(STATUS "Registered ${TEST_COUNT} individual Python tests")
        message(STATUS "Run Python tests with: ctest -L python")
    else()
        message(WARNING "pytest not found - Install with: pip3 install pytest numpy")
    endif()
else()
    message(WARNING "Python3 not found - Python tests will not be available")
endif()
