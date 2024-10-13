cmake_minimum_required(VERSION 3.1)

project (putdisorted)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(putdisorted main.cpp)
target_link_libraries(putdisorted ${OpenCV_LIBS})