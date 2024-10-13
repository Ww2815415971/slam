cmake_minimum_required(VERSION 3.1)
project (testceres)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
include_directories( "/usr/include/eigen3" )
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
add_executable(testceres main.cpp)
target_link_libraries(testceres ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
