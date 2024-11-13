cmake_minimum_required(VERSION 3.1 )
project(pose_estimation)

find_package(OpenCV REQUIRED)
include_directories(pose_estimation ${OpenCV_INCLUDE_DIRS})
add_executable(pose_estimation_2d_2d pose_estimation_2d_2d.cpp)
target_link_libraries(pose_estimation_2d_2d ${OpenCV_LIBS})