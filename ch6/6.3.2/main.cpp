#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;
struct CURVE_FITTING_COST{

    CURVE_FITTING_COST(double x,double y) :_x(x), _y(y) {} // 构造函数
    // 残差的计算
    template <typename T> // 任意类型变量传入都可以计算
    bool operator()(const T * const abc,
    T * residual) const {
        // y - exp (ax`2+bx+c)
        residual [0] = T(_y) - ceres::exp(abc[0] * T(_x)*(_x) + abc[1] * T(_x)  + abc[2]);
        return true;
    }
    const double _x,_y;
};
int main(int argc, char ** argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0; // 真实参数值
    double ae = 2.0, be = -1.0,ce = 5.0;// 预测初始参数值
    int N = 100;          // 数据点
    double w_sigma =1.0;  // 噪声值
    double inv_sigma = 1.0 / w_sigma; // 取倒数
    cv::RNG rng;
    vector <double> x_data,y_data;
    for (int  i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr ) + rng.gaussian(w_sigma * w_sigma)); // 真实值 + 噪声值(按照高斯分布来进行随机）)
    }
    double abc[3] = {ae,be,ce};
    // 构建估计参数值
    
    // 构建最小二乘问题
    ceres::Problem problem;
    for(int i = 0 ; i < N; i ++ )
    {
        problem.AddResidualBlock( // 向问题添加误差项
            // 使用自动求导，模版参数，误差类型，输出维度，输入维度，维数与 struct 中保持一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
                new CURVE_FITTING_COST(x_data[i],y_data[i])
        ),
        nullptr, // 核函数
        abc  // 待估计参数
        );
    }


    // 配置求解器
    ceres:: Solver:: Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; // 增量方程如何求解，使用CHOLESKY 的方法来进行求解
    options.minimizer_progress_to_stdout = true; // 将每次的迭代信息进行输出
    ceres::Solver::Summary summary; // 优化信息
    chrono ::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options,&problem,&summary);//开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solver time cost = " <<time_used.count() << "seconds"<<endl;


    // 输出
    cout << summary.BriefReport()<< endl;
    cout << "estimated a,b,c= ";
    for(auto a:abc ) cout << a<<" ";
    cout<< endl;
    return 0; 
}