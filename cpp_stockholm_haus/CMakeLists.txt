cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# "../" --> output directory relative same to running in IDE
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)

if (MSVC)
  set(CMAKE_CXX_STANDARD 17)
endif()

project(stockholm)

include_directories(.)


# add WIN32 to remove console window ->
add_executable (${CMAKE_PROJECT_NAME} #WIN32
stockholm.cpp
ransac_plane.hpp
pointcloud_io.hpp
)
