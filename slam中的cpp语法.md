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

virtual // 虚函数的定义
override // 重写基类(父类)虚函数
public // 设置变量使用范围
```
## explicit 
- explicit
    
        防止隐式转换
## 报错
    纯虚拟 函数 "g2o::OptimizableGraph::Vertex::write" 没有强制替代项