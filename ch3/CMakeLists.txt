cmake_minimum_required( VERSION 3.1)

project(SHOWTrajectory)

# 设置 C++ 参数值标准
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 连接eigen     
include_directories( "/usr/include/eigen3" )
#  添加 epoxy 包
find_package(PkgConfig REQUIRED)

include_directories(${EPOXY_INCLUDE_DIRS})
# 查找 glew 包
find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "GLEW not found")
endif()
include_directories(${GLEW_INCLUDE_DIRS})

# 查找 Pangolin 包

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


# 添加可执行文件
add_executable(showtrajectory main.cpp)



# 链接 Pangolin 库
target_link_libraries(showtrajectory ${EPOXY_LIBRARIES} ${GLEW_LIBRARIES} ${Pangolin_LIBRARIES} )


