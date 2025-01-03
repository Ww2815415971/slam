# 学习与调试记录

## bug记录模块

本来想用<视觉slam 12讲>p67 代码来总结，遇到以下bug
### 1.无法找到#include <Eigen/Core>
解决方案：将Eigen/Core 改成 
```cpp
#include <eigen3/Eigen/Core>
```
### 2. 未定义 Isometry3d类型
解决方案：添加
```cpp
#include <eigen3/Eigen/Geometry>
```
### 3.CMakeLists.txt 的配置
```
cmake_minimum_required( VERSION 3.1)

project(SHOWTrajectory)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 查找 Pangolin 包
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# 查找 Eigen 包
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# 添加可执行文件
add_executable(showtrajectory main.cpp)

# 链接 Pangolin 库
target_link_libraries(showtrajectory &{Pangolin_LIBRARIES})
target_link_libraries(showtrajectory Eigen3:: Eigen)
```
### 4.编译视觉slam12讲的代码报错记录
#### 报错1
```
/usr/include/epoxy/gl.h:38:2: error: #error epoxy/gl.h must be included before (or in place of) GL/gl.h
   38 | #error epoxy/gl.h must be included before (or in place of) GL/gl.h
      |  ^~~~~
      epoxy 是一个 OpenGL 处理库，用于解决 OpenGL 实现的兼容性问题。
```
- 解决方案：
    导入#include <epoxy/gl.h>在CMakeLists.txt中也添加对应的库,而且要注意导入库的顺序，需要和target_link_libiaries()相对应
#### 报错2
-    如果出现很多报错的情况，需要更新头文件，因为头文件不兼容的问题，暂时找不到原因
```
   
      In file included from /home/steven/slam/slambook/ch3/main.cpp:2:
/usr/include/GL/glew.h:21142:28: error: conflicting declaration ‘typedef void (* PFNGLFRAGMENTLIGHTMODELIVSGIXPROC)(GLenum, GLint*)’
21142 | typedef void (GLAPIENTRY * PFNGLFRAGMENTLIGHTMODELIVSGIXPROC) (GLenum pname, GLint* params);
      |                            ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/GL/gl.h:2050,
                 from /home/steven/slam/slambook/ch3/main.cpp:1:
/usr/include/GL/glext.h:12303:25: note: previous declaration as ‘typedef void (* PFNGLFRAGMENTLIGHTMODELIVSGIXPROC)(GLenum, const GLint*)’
12303 | typedef void (APIENTRYP PFNGLFRAGMENTLIGHTMODELIVSGIXPROC) (GLenum pname, const GLint *params);
      |                         ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /home/steven/slam/slambook/ch3/main.cpp:2:
/usr/include/GL/glew.h:21144:28: error: conflicting declaration ‘typedef void (* PFNGLFRAGMENTLIGHTFVSGIXPROC)(GLenum, GLenum, GLfloat*)’
21144 | typedef void (GLAPIENTRY * PFNGLFRAGMENTLIGHTFVSGIXPROC) (GLenum light, GLenum pname, GLfloat* params);
```
- 解决方案
    依照这种头文件格式
    ```cpp
    #include <epoxy/gl.h>     
    #include <GL/gl.h>        // 随后添加 OpenGL 头文件
    #include <pangolin/pangolin.h>
    #include <eigen3/Eigen/Core>
    #include <eigen3/Eigen/Geometry>
    #include <unistd.h>
    #include <fstream>
    #include <iostream>
    ```
#### 报错3
    在进行cmake的时候无法找到EGL
    CMake Error at CMakeLists.txt:13 (find_package):
    By not providing "Findfmt.cmake" in CMAKE_MODULE_PATH this project has
    asked CMake to find a package configuration file provided by "fmt", but
    CMake did not find one.

     Could not find a package configuration file provided by "fmt" with any of
    the following names:

     fmtConfig.cmake
    fmt-config.cmake

    Add the installation prefix of "fmt" to CMAKE_PREFIX_PATH or set "fmt_DIR"
    to a directory containing one of the above files.  If "fmt" provides a
    separate development package or SDK, be sure it has been installed.
- 解决方案
    - 找到EGL的路径在/usr/include/EGL
#### 报错4
     异常关闭挂起的ubuntu导致无法识别ens33 网卡
  解决方案 
-     sudo systemctl stop NetworkManager
-     sudo rm /var/lib/NetworkManager/NetworkManager.state
-     sudo systemctl start NetworkManager
### 提问：为什么链接Pangolin库和连接Eigen 库的语句不一样

解答：不同的库之所以会有不同的连接方式。这主要取决于库的 CMake 配置文件（通常是 find_package 模块）是如何定义和导出库的编译和链接信息的

##  学习模块
### 装载库文件
    sudo make install 
- 安装路径  
  - /usr/include
### 如果需要更改配置发现vscode不工作需要刷新
     重启 VSCode 或 IntelliSense
    在命令面板中 (Ctrl+Shift+P)，输入 C/C++: Restart IntelliSense Engine，然后选择它以重启 IntelliSense。
    如果仍然不工作，尝试重启 VSCode。
### vscode配置cmake环境找不到头文件
    下载cmake-tools
    ctrl shift +p 进入命令行
    选择
![My Image](images/cmake.png)
- 第一个是去直接去makeCMake工程
- 第二个是让vscode去知道main.cpp文件中的头文件与CMakelists.txt的对应关系，避免报错
### c++知识

#### 重载
- 函数重载
    
  多个重名函数通过参数进行不同的调用
- 运算符重载

  将只能对数字类型进行处理的功能拓展到对各种类型的处理的拓展

 ### openGL介绍：
     可以跨平台的一个显示图像的工具，在任何的图像处理和显示方面都有应用
 ## 视觉SLAM十四讲李群与李代数
 需要对相机位置进行估计与优化，在什么状态下可以得到符合当前的观测数据，就是在发生转弯过程中，如何保证相机与所需要的观测数据保证一致。位姿与观测数据形成的目标函数估计与优化，进行求导。李群对乘法封闭，李代数对加法封闭
 ### 知识体系构建

- SO(3)与SE(3)
  
  - SO(3)是特殊正交群，是三维旋转矩阵
  - SE(3)是特殊欧式群，变换矩阵，结合了旋转和平移的4*4的矩阵，
  - 它们对加法都不封闭，对乘法封闭，它们相加还是不是一个群，它们相乘还是一个群
- 李群，由于李群不能相加，所以根据定义无法取极限，无法求导
  
  是一个集合加上一种运算的代数结构
  - 性质
    - 封闭性
    - 结合律
    - 幺元

      单位元：
      存在一个a1属于一种规则，使得任意一个属于一种规则的a，使a1 * a=a * a1=a。
      单位矩阵
    - 逆
- 李代数

  对应不同李群的正切空间，描述李群的局部导数
  记作se(3)和so(3)
 ## ubuntu常见网络报错
 - 无法显示网络图标
   由于异常关闭虚拟机，导致系统无法识别网卡，从而导致出现没有网络图标的情况
 - 解决方案
    ```sh
    sudo systemctl stop NetworkManager
sudo rm /var/lib/NetworkManager/NetworkManager.state
sudo systemctl start NetworkManager
    ```
## ch7/7.8 报错
    
     /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: warning: relocation against `_ZN3g2o18G2OBatchStatistics12_globalStatsE' in read-only section `.text._ZN3g2o18G2OBatchStatistics11globalStatsEv[_ZN3g2o18G2OBatchStatistics11globalStatsEv]'
     /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `bundleAdjustmentG2O(std::vector<Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1, 0, 3, 1> > > const&, std::vector<Eigen::Matrix<double, 2, 1, 0, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 2, 1, 0, 2, 1> > > const&, cv::Mat const&, Sophus::SE3<double, 0>&)':
     pnp.cpp:(.text+0x25c0): undefined reference to `g2o::OptimizationAlgorithmGaussNewton::OptimizationAlgorithmGaussNewton(std::unique_ptr<g2o::Solver, std::default_delete<g2o::Solver> >)'
     /usr/bin/ld: pnp.cpp:(.text+0x2603): undefined reference to `g2o::SparseOptimizer::SparseOptimizer()'
     /usr/bin/ld: pnp.cpp:(.text+0x261c): undefined reference to `g2o::SparseOptimizer::setAlgorithm(g2o::OptimizationAlgorithm*)'
     /usr/bin/ld: pnp.cpp:(.text+0x2630): undefined reference to `g2o::SparseOptimizer::setVerbose(bool)'
     /usr/bin/ld: pnp.cpp:(.text+0x2902): undefined reference to `g2o::HyperGraph::Edge::setId(int)'
     /usr/bin/ld: pnp.cpp:(.text+0x299a): undefined reference to `g2o::OptimizableGraph::addEdge(g2o::OptimizableGraph::Edge*)'
     /usr/bin/ld: pnp.cpp:(.text+0x29ea): undefined reference to `g2o::SparseOptimizer::setVerbose(bool)'
     /usr/bin/ld: pnp.cpp:(.text+0x29fe): undefined reference to `g2o::SparseOptimizer::initializeOptimization(int)'
     /usr/bin/ld: pnp.cpp:(.text+0x2a17): undefined reference to `g2o::SparseOptimizer::optimize(int, bool)'
     /usr/bin/ld: pnp.cpp:(.text+0x2a97): undefined reference to `g2o::SparseOptimizer::~SparseOptimizer()'
     /usr/bin/ld: pnp.cpp:(.text+0x2b70): undefined reference to `g2o::SparseOptimizer::~SparseOptimizer()'
     /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::OptimizableGraph::addVertex(g2o::OptimizableGraph::Vertex*)':
     pnp.cpp:(.text._ZN3g2o16OptimizableGraph9addVertexEPNS0_6VertexE[_ZN3g2o16OptimizableGraph9addVertexEPNS0_6VertexE]+0x28): undefined reference to `g2o::OptimizableGraph::addVertex(g2o::OptimizableGraph::Vertex*, g2o::HyperGraph::Data*)'
     /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::G2OBatchStatistics::globalStats()':
     pnp.cpp:(.text._ZN3g2o18G2OBatchStatistics11globalStatsEv[_ZN3g2o18G2OBatchStatistics11globalStatsEv]+0xb): undefined reference to `g2o::G2OBatchStatistics::_globalStats'
     /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::BlockSolverBase::~BlockSolverBase()':
     pnp.cpp:(.text._ZN3g2o15BlockSolverBaseD2Ev[_ZN3g2o15BlockSolverBaseD5Ev]+0x26): undefined reference to `g2o::Solver::~Solver()'
     /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::BaseVertex<6, Sophus::SE3<double, 0> >::~BaseVertex()':
    pnp.cpp:(.text._ZN3g2o10BaseVertexILi6EN6Sophus3SE3IdLi0EEEED2Ev   [_ZN3g2o10BaseVertexILi6EN6Sophus3SE3IdLi0EEEED5Ev]+0x47): undefined reference to `g2o::OptimizableGraph::Vertex::~Vertex()'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::BaseEdge<2, Eigen::Matrix<double, 2, 1, 0, 2, 1> >::~BaseEdge()':
    pnp.cpp:(.text._ZN3g2o8BaseEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEEED2Ev[_ZN3g2o8BaseEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEEED5Ev]+0x35): undefined reference to `g2o::OptimizableGraph::Edge::~Edge()'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::BaseVertex<6, Sophus::SE3<double, 0> >::BaseVertex()':
    pnp.cpp:(.text._ZN3g2o10BaseVertexILi6EN6Sophus3SE3IdLi0EEEEC2Ev[_ZN3g2o10BaseVertexILi6EN6Sophus3SE3IdLi0EEEEC5Ev]+0x28): undefined reference to `g2o::OptimizableGraph::Vertex::Vertex()'
    /usr/bin/ld: pnp.cpp:(.text._ZN3g2o10BaseVertexILi6EN6Sophus3SE3IdLi0EEEEC2Ev[_ZN3g2o10BaseVertexILi6EN6Sophus3SE3IdLi0EEEEC5Ev]+0xe0): undefined reference to `g2o::OptimizableGraph::Vertex::~Vertex()'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::BlockSolverBase::BlockSolverBase()':
    pnp.cpp:(.text._ZN3g2o15BlockSolverBaseC2Ev[_ZN3g2o15BlockSolverBaseC5Ev]+0x18): undefined reference to `g2o::Solver::Solver()'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o: in function `g2o::BaseEdge<2, Eigen::Matrix<double, 2, 1, 0, 2, 1> >::BaseEdge()':
    pnp.cpp:(.text._ZN3g2o8BaseEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEEEC2Ev[_ZN3g2o8BaseEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEEEC5Ev]+0x19): undefined reference to `g2o::OptimizableGraph::Edge::Edge()'
    /usr/bin/ld: pnp.cpp:(.text._ZN3g2o8BaseEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEEEC2Ev[_ZN3g2o8BaseEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEEEC5Ev]+0x8c): undefined reference to `g2o::OptimizableGraph::Edge::~Edge()'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTV14EdgeProjection[_ZTV14EdgeProjection]+0x40): undefined reference to `g2o::OptimizableGraph::Edge::setMeasurementData(double const*)'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTV14EdgeProjection[_ZTV14EdgeProjection]+0x48): undefined reference to `g2o::OptimizableGraph::Edge::getMeasurementData(double*) const'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTV14EdgeProjection[_ZTV14EdgeProjection]+0x50): undefined reference to `g2o::OptimizableGraph::Edge::measurementDimension() const'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTV14EdgeProjection[_ZTV14EdgeProjection]+0x58): undefined reference to `g2o::OptimizableGraph::Edge::setMeasurementFromState()'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTV14EdgeProjection[_ZTV14EdgeProjection]+0xd8): undefined reference to `g2o::OptimizableGraph::Edge::resolveCaches()'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTVN3g2o13BaseUnaryEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEE10VertexPoseEE[_ZTVN3g2o13BaseUnaryEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEE10VertexPoseEE]+0x40): undefined reference to `g2o::OptimizableGraph::Edge::setMeasurementData(double const*)'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTVN3g2o13BaseUnaryEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEE10VertexPoseEE[_ZTVN3g2o13BaseUnaryEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEE10VertexPoseEE]+0x48): undefined reference to `g2o::OptimizableGraph::Edge::getMeasurementData(double*) const'
    /usr/bin/ld: CMakeFiles/pnp.dir/pnp.cpp.o:(.data.rel.ro._ZTVN3g2o13BaseUnaryEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEE10VertexPoseEE[_ZTVN3g2o13BaseUnaryEdgeILi2EN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEE10VertexPoseEE]+0x50): undefined reference to `g2o::OptimizableGraph::Edge::measurementDimension() const'
