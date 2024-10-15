cmake_minimum_required(VERSION 3.1)
project(orbcv)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )
add_executable(orbcv orbcv.cpp)
target_link_libraries(orbcv ${OpenCV_LIBS})