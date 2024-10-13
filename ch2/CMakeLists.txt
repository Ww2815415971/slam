cmake_minimum_required(VERSION 3.1)

project (HelloSLAM)

include_directories(/home/steven/slam/slambook/ch2)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)    # 静态库
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)    # 动态库
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)    # 可执行文件
add_executable(helloSLAM helloSLAM.cpp usehello.cpp)
#target_link_libraries(helloSLAM /home/steven/slam/slambook/ch2/libhelloslam)