cmake_minimum_required(VERSION 3.0)
project(TrajectoryError)

# 包含路径
include_directories(/usr/include/EGL)
include_directories(/usr/include)

# 库路径
link_directories(/usr/lib/x86_64-linux-gnu)

# 查找其他包（例如 Pangolin 和 fmt）
find_package(Pangolin REQUIRED)
find_package(fmt REQUIRED)

# 添加可执行文件
add_executable(trajectoryError trajectoryError.cpp)

# 链接库
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${fmt_LIBRARIES} EGL)
