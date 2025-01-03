# cpp 高级语法
## assert

        可以对传入的表达式进行判断，如果错误会在执行代码中提示
## 类的语法
### 继承
```cpp

    class CurveFittingVertex : public g2o::BaseVertex<3 ,Eigen::Vector3d>
```
        继承 g2o::BaseVertex类
### 虚函数
```cpp

- virtual // 虚函数的定义
```

```cpp
        virtual void setToOriginImpl()override {
            _estimate = Sophus::SE3d();
        }//去通过重写虚函数方法让子类能够实现特定的逻辑

        override // 重写基类(父类)虚函数
        public // 设置变量使用范围
```

### explicit 
- explicit
    
        防止隐式转换
## 报错
    纯虚拟 函数 "g2o::OptimizableGraph::Vertex::write" 没有强制替代项

## incline
内联函数
   
   适合那些短小逻辑较为简单的语句，不去消耗栈空间通过栈帧去执行函数，而是直接类似于定义这种顺序执行
## 使用Eigen库中的知识
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    是在Eigen库中去进行数据内存对齐的一个声明，在C++中一般是使用new运算符分配内存，默认情况下不保证对齐内存分配，某些库如Eigen库对于特定类型的对象要求对齐内存分配，以便于处理器更高效的加载和存储数据，提高效率，从而实现大规模数据处理，并行计算和使用SIMD指令进行矢量化计算
### tips
- 矢量化计算与SIMD指令(单指令多数据)
              
        矢量化计算，意为使用矩阵和向量进行运算，使用现代处理器的指令如(Intel的AVX 
        SSE指令集和AMD的SIMD指令)可以在一个指令中一个时钟周期中处理多个数据
## static_cast

    将基类指针转化成派生类指针
### 基类与派生类

    等同于父类与子类，就是需要去构建一个更加安全的派生类对象