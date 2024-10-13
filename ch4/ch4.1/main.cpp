#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "sophus/se3.hpp"
using namespace std;
using namespace Eigen;

int main(int argc,char **argv)
{

    //        对SO(3)操作
    //            角轴       角度        以谁为轴
    Matrix3d R = AngleAxisd(M_PI /2,Vector3d(0,0,1)).toRotationMatrix();  // 以谁为轴转动多少角度，表示这个变化的矩阵
    Quaterniond q(R);
    Sophus :: SO3d SO3_R(R);    // 也可以从旋转矩阵中构造
    Sophus :: SO3d SO3_q(q);    // 从四元数中构造
    // 等价
    cout << "SO(3) from matrix:\n"<< SO3_R.matrix()<< endl;
    cout << "SO(3) from quarterion : \n" << SO3_q.matrix() << endl;
    cout << "they are equal:\n" << endl;
    // 根据对数映射获得李代数
    Vector3d so3=SO3_R.log();
    cout << "so3= " << so3.transpose() << endl;  // 进行转置
    // hat 为向量到反对称矩阵
    cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) <<endl;
    // vww 为反对称到向量
    cout << "so3 hat vee= " << Sophus::SO3d::vee (Sophus :: SO3d :: hat(so3).transpose()) << endl;
    Vector3d update_so3(1e-4, 0, 0); // 迭代更新量
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 update = \n" << SO3_updated.matrix() << endl;
    cout << "*******************************************" << endl;
    //   对SE(3)操作
    Vector3d t (1, 0, 0);           // 沿 X 轴平移 1
    Sophus :: SE3d SE3_Rt(R,t);     // 从 R,t 构造SE(3)
    Sophus::SE3d SE3_qt(q,t);
    cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
    cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
    //     李代数se(6)是一个六维向量
    typedef  Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 =SE3_Rt.log();
    cout << "se3= " << se3.transpose() << endl;
    cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 hat vee= " << Sophus::SE3d::vee (Sophus :: SE3d :: hat(se3).transpose()) << endl;

    // 更新
    Vector6d  update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_update = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 update = " << endl << SE3_update.matrix() << endl;
    return 0;


}