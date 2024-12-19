cmake_minimum_required(VERSION 3.1)

project(pnp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_PARFIX_PATH "/usr/local/g2o")# 非必须
# set(G2O_INCLUDE_DIRS "/usr/local/g2o/include")#非必须
# set(G2O_LIB_DIRS "/usr/local/g2o/lib")#菲必须

include_directories( "/usr/include/eigen3")
include_directories("/usr/local/g2o/include")
# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen3
find_package(Sophus REQUIRED)
# g2o
#find_package(G2O REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")# 必须
#set(G2O_INCLUDE_DIRS "/usr/local/g2o/include")

# find_package(G2O REQUIRED)# 菲必须
#message(STATUS "G2O_INCLUDE_DIRS: ${G2O_INCLUDE_DIRS}")

link_directories("/usr/local/g2o/lib")# 最重要的一句指定编译器去指定目录寻找，用来解决

add_executable(pnp pnp.cpp)
# target_link_libraries(pnp ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} )
target_link_libraries(pnp ${OpenCV_LIBS} g2o_core g2o_stuff )


# cmake_minimum_required(VERSION 2.8)
# project(pnp)

# set(CMAKE_BUILD_TYPE "Release")
# add_definitions("-DENABLE_SSE")
# set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find_package(OpenCV 3 REQUIRED)
# find_package(G2O REQUIRED)
# find_package(Sophus REQUIRED)

# include_directories(
#         ${OpenCV_INCLUDE_DIRS}
#         ${G2O_INCLUDE_DIRS}
#         ${Sophus_INCLUDE_DIRS}
#         "/usr/include/eigen3/"
# )





# add_executable(pnp pnp.cpp)
# target_link_libraries(pnp
#         g2o_core g2o_stuff
        # ${OpenCV_LIBS})


