cmake_minimum_required(VERSION 3.10)
project(icp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


include_directories("/usr/include/eigen3")
include_directories("/usr/local/g2o/include")
# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")# 必须
# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Find Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

link_directories("/usr/local/g2o/lib")
# Add executable
add_executable(${PROJECT_NAME} icp.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} g2o_core g2o_stuff)