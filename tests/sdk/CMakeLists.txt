cmake_minimum_required(VERSION 3.15)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
# Include GoogleTest module
include(GoogleTest)

# Include shared test utilities
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AditofTest.cmake)

add_aditof_test(generic-sdk_version_test generic/generic-sdk_version_test.cpp)
add_aditof_test(system-get_camera_list system/system-get_camera_list.cpp)
add_aditof_test(camera-get_frames camera/camera-get_frames.cpp)
add_aditof_test(camera-adsd3500_reset camera/camera-adsd3500_reset.cpp)
add_aditof_test(libaditof_sample_test sample/sample_test.cpp)

# Camera parameters test
add_aditof_test(camera-parameters camera/camera-parameters.cpp)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/camera/data/camera-parameters/)
    file(COPY camera/data/camera-parameters/ DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()