cmake_minimum_required(VERSION 3.0)

add_subdirectory(pybind11)

include_directories(${LIBADITOF_SUBMODULE_PATH}/sdk/includes)

pybind11_add_module(aditofpython aditofpython.cpp)

target_link_libraries(aditofpython PRIVATE aditof)

# Copy the built .so file to each example folder in examples/bindings/python
set(EXAMPLE_DIRS 
    "dnn" 
    "first_frame" 
    "gesture_rec" 
    "streaming" 
    "skeletal_tracking" 
    "skeletal_tracking_in_pointcloud" 
    "deprecated/saveCCBToFile" 
    "data_collect" 
    "showPointCloud"
)

foreach(EXAMPLE_DIR ${EXAMPLE_DIRS})
    add_custom_command(TARGET aditofpython POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:aditofpython>"
        "${CMAKE_BINARY_DIR}/examples/bindings/python/${EXAMPLE_DIR}/"
        COMMENT "Copying aditofpython .so to examples/bindings/python/${EXAMPLE_DIR}"
    )
endforeach()

get_target_property(target_prefix aditofpython PREFIX)
get_target_property(target_suffix aditofpython SUFFIX)
set(TARGET_OUTPUT_NAME "${target_prefix}aditofpython${target_suffix}")

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/setup.py.cmakein
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py
)

if(NOT WIN32)
        install(CODE "execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND ${PYTHON_EXECUTABLE} setup.py install --prefix=${CMAKE_INSTALL_PREFIX})")
endif()
