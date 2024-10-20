cmake_minimum_required(VERSION 3.1)
project(orb_self)

find_package(OpenCV REQUIRED)
include_directories(orb_self ${OpenCV_INCLUDE_DIRS})
add_executable(orb_self orb_self.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mpopcnt")

target_link_libraries(orb_self ${OpenCV_LIBS})