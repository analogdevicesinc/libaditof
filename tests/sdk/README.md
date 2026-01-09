# SDK Tests

## Adding New Tests

All test directories use a shared helper function pattern to keep `CMakeLists.txt` files clean and maintainable.

### Basic Usage

To add a new test, add one line to your test directory's `CMakeLists.txt`:

```cmake
add_aditof_test(test-name source_file.cpp)
```

### Setup for New Test Directory

Each test directory needs only these 3 lines of boilerplate:

```cmake
cmake_minimum_required(VERSION 3.15)

include(GoogleTest)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/AditofTest.cmake)

# Then add your tests...
add_aditof_test(your-test-name your_test.cpp)
```

### Examples

**Single file test:**
```cmake
add_aditof_test(libaditof_sample_test sample_test.cpp)
```

**Multi-file test:**
```cmake
add_aditof_test(sample-complex_test 
    complex_test.cpp 
    helpers.cpp 
    mocks.cpp
)
```

**Test with custom dependencies:**
```cmake
add_aditof_test(sample-special_test special_test.cpp)
target_link_libraries(sample-special_test PRIVATE extra_lib)
```

**Test with custom compile options:**
```cmake
add_aditof_test(sample-optimized_test optimized_test.cpp)
target_compile_options(sample-optimized_test PRIVATE -O3)
```

**Test with custom output directory:**
```cmake
add_aditof_test(sample-placed_test placed_test.cpp)
set_target_properties(sample-placed_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/tests/sample
)
```

## Benefits

- **One line per test** - Easy to scan and maintain
- **Consistent configuration** - All tests use same dependencies and includes
- **Easy to customize** - Function handles boilerplate, customize after if needed
- **Scales well** - Add 50 tests with 50 lines instead of 500+
- **Shared function** - Update behavior in one place (`tests/cmake/AditofTest.cmake`)

## Alternative: Auto-discover Pattern

For even more automation, you can scan for `*_test.cpp` files:

```cmake
file(GLOB TEST_SOURCES "*_test.cpp")
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_aditof_test(sample-${TEST_NAME} ${TEST_SOURCE})
endforeach()
```

⚠️ **Caution:** Auto-discovery means CMake won't re-configure when you add new files. You must manually re-run `cmake` or touch `CMakeLists.txt`.

## Directory Structure

```
tests/sdk/
├── cmake/
│   └── AditofTest.cmake       ← Shared helper function
├── sample/
│   ├── CMakeLists.txt          ← 3 lines + test definitions
│   └── sample_test.cpp
├── generic/
│   ├── CMakeLists.txt
│   └── generic-sdk_version_test.cpp
└── camera/
    ├── CMakeLists.txt
    └── ...
```
