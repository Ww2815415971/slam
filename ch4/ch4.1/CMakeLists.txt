cmake_minimum_required( VERSION 3.1)

project(SOSE)

find_package(Sophus REQUIRED)
include_directories("/usr/include/eigen3")

add_executable(SOSE main.cpp)