# 奇异值分解原理

是一种矩阵因子分解方法，是一种数据降维的经典方法。
## 例子
    通过分解矩阵来发现矩阵表示成数组元素时不明显的函数性质，将矩阵分解成一组特征向量和特征值，分析矩阵的特定性质
## 引出奇异值分解
    通过奇异值分解得到一些与特征分解相同类型的信息，对每一个实数矩阵都有一个奇异值分解，但不一定有特征分解(非方阵矩阵)

## 特征分解
    是将可对角化的方阵分解成特征值构成的向量和特征向量构成的矩阵
## 奇异值分解
    将任何的矩阵分解成三个矩阵的乘积
 $$
    A = U \Sigma V^T
$$
    U是m阶正交矩阵代表旋转，V是n阶正交矩阵代表拉伸,
 $$  \Sigma $$
 是降序排列的对角线元素组成的m * n 矩形对角矩阵代表旋转


## 如何计算SVD分解

求M^T*M的特征向量得到V
求M*M^T的特征向量得到U
求M^T*M或M*M^T的特征值，然后开方得到奇异值 构成对角阵 $ \Sigma$

## slam里的SVD分解

    设置误差，利用误差构建最小化二乘问题，对最小二乘问题进行求导，
    导数为0得到平移矩阵与两张图片的点的误差和的方程，然后带回到原最小二乘问题的函数中，根据最小二乘问题只变为关于R变量的一个方程，然后得到W
    通过SVD分解W解出最优的R，
    通过R来求t
    详解关注https://gene2022.blog.csdn.net/article/details/134131996
    虽然我看不懂。
