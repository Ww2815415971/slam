cmake_minimum_required(VERSION 3.1)

project(gassNewton)

include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(gassNewton main.cpp)
target_link_libraries(gassNewton ${OpenCV_LIBS})