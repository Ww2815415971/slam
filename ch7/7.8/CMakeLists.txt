cmake_minimum_required(VERSION 3.1)

project(pnp)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen3
include_directories("/usr/include/eigen3")

# g2o
include_directories("/usr/local/g2o/include")
message(STATUS "G2O_INCLUDE_DIRS: ${G2O_INCLUDE_DIRS}")

find_package(Sophus REQUIRED)


add_executable(pnp pnp.cpp)
target_link_libraries(pnp ${OpenCV_LIBS} ${G2O_CORE_LIBRARY})