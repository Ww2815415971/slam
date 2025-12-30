# g2o

论文网站  [text](https://mengwenhe-cmu.github.io/Reading-Reports/Research/Localization/Graph_Optimization/g2o_A_General_Framework_for_Graph_Optimization/paper.pdf)

- g2o与神经网络的区别

g2o 是一个图优化框架，图中的每个节点表示一个变量（如相机位置，3D点位置）每条边表示节点之间的约束关系，通过最小边定义的误差函数去优化图中的所有节点，通常为非线性最小二乘优化，保证最后每条边的误差函数值都为最小，相当于全局最优化。

神经网络，通过前向传播得到输出，计算损失，通过损失函数去计算网络输出真实标签之间的差异，反向传播，根据损失函数的梯度，去求得每个参数的梯度。参数更新 使用梯度下降，使得损失函数的值最小。相当于是全局最小化。



/usr/bin/ld: 找不到 -lg2o_core: 没有那个文件或目录
/usr/bin/ld: 找不到 -lg2o_stuff: 没有那个文件或目录


link_directories("/usr/local/g2o/lib")#
