cmake_minimum_required(VERSION 3.15)

# Add C++ tests (Google Test)
add_subdirectory(sdk)

# Add Python tests (pytest) if Python bindings are enabled
if(WITH_PYTHON)
    add_subdirectory(python)
endif()
