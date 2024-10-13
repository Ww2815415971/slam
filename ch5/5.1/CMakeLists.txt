cmake_minimum_required( VERSION 3.1)


project(opencv)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(opencv main.cpp)
target_link_libraries(opencv ${OpenCV_LIBS})