cmake_minimum_required(VERSION 3.0)
project(dependecies)

add_subdirectory(third-party/c_json)
add_subdirectory(third-party/crc32)
if (WITH_NETWORK_COMPRESSION)
	add_subdirectory(third-party/lz4)
endif()
