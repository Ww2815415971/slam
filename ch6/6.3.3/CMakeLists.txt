cmake_minimum_required(VERSION 3.1)
project(testg2o)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
#set(CMAKE_PREFIX_PATH "/usr/local/g2o")

# eigen3

include_directories( "/usr/include/eigen3" )
include_directories("/usr/local/g2o/include")
#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})



#g2o
set(G2O_INCLUDE_DIRS "/usr/local/g2o/include")
list(APPEND CMAKE_MODULE_PATH /home/steven/github/g2o/cmake_modules)
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})
message(STATUS "G2O_INCLUDE_DIRS: ${G2O_INCLUDE_DIRS}")
#message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
add_executable(testg2o main.cpp)

target_link_libraries(testg2o ${OpenCV_LIBS}  ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} )
