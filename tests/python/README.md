# Python Bindings Tests

This directory contains comprehensive pytest tests for the aditofpython bindings.

## Overview

The test suite validates the Python bindings by testing:

- **Version API**: Version string retrieval functions
- **System API**: Camera enumeration and system initialization
- **Camera API**: Camera operations (initialize, set mode, start, stop)
- **Frame API**: Frame data structures and access
- **Sensor Interface**: Sensor callback registration
- **Data Access**: Frame data retrieval and numpy integration
- **Integration Scenarios**: Real-world usage workflows

## Files

- `conftest.py` - Pytest configuration and shared fixtures
- `test_aditof_bindings.py` - Comprehensive test suite for aditofpython
- `requirements.txt` - Python dependencies

## Setup

### Install Dependencies

```bash
pip install -r requirements.txt
```

### Build the Python Bindings

The Python bindings must be built before tests can run. This is done through the CMake build system:

```bash
cd /path/to/build
cmake -DWITH_PYTHON=ON ..
make
```

The compiled `aditofpython` module will be available in the build directory.

## Running Tests

### Run All Tests

```bash
pytest -v
```

### Run Specific Test Classes

```bash
# Test version API
pytest tests/python/test_aditof_bindings.py::TestVersionAPI -v

# Test System API
pytest tests/python/test_aditof_bindings.py::TestSystemAPI -v

# Test Camera API
pytest tests/python/test_aditof_bindings.py::TestCameraAPI -v
```

### Run Tests with Different Verbosity Levels

```bash
# Verbose output
pytest -v

# Very verbose with extra information
pytest -vv

# Quiet output
pytest -q
```

### Run Tests and Show Skipped Tests

```bash
pytest -v -rs
```

### Run Tests by Pattern

```bash
# Run tests matching "camera"
pytest -k "camera" -v

# Run tests matching "frame" but not "details"
pytest -k "frame and not details" -v
```

## Test Categories

### Hardware-Independent Tests

These tests validate the API without requiring physical hardware:

- Version API tests
- Enum and structure tests
- Module import tests

Run with:

```bash
pytest -k "Version or Status or Details or Structure" -v
```

### Hardware-Dependent Tests

These tests require camera hardware or will be skipped:

- Camera initialization
- Frame request
- Sensor operations

These tests use pytest's `skip` mechanism and will show as "SKIPPED" if hardware is unavailable.

## Fixtures

The test suite uses several pytest fixtures defined in `conftest.py`:

- `system` - A System instance
- `camera_list` - List of available cameras
- `camera` - First available camera (skips if none available)
- `initialized_camera` - An initialized camera (skips if initialization fails)
- `frame` - A Frame instance

## Test Coverage

To generate coverage reports:

```bash
pip install pytest-cov
pytest --cov=aditofpython tests/python/ --cov-report=html
```

Then open `htmlcov/index.html` in your browser.

## Troubleshooting

### ImportError: No module named aditofpython

Make sure the Python bindings are built and the module is in the Python path:

```bash
export PYTHONPATH=/path/to/build:$PYTHONPATH
pytest -v
```

Or install the bindings:

```bash
cd /path/to/build
python setup.py install
pytest -v
```

### Tests are Skipped

If tests are being skipped, it usually means:

1. Camera hardware is not available (this is normal for CI/CD)
2. The aditofpython module is not properly built
3. Camera initialization is failing due to configuration issues

Check the output with `-rs` flag to see skip reasons:

```bash
pytest -v -rs
```

## Integration with CMake

To run Python tests as part of the CMake build process, add to `CMakeLists.txt`:

```cmake
if(BUILD_TESTING AND WITH_PYTHON)
    find_program(PYTEST_EXECUTABLE pytest)
    if(PYTEST_EXECUTABLE)
        add_test(
            NAME python_bindings_tests
            COMMAND pytest -v ${CMAKE_CURRENT_SOURCE_DIR}/tests/python/
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
    endif()
endif()
```
