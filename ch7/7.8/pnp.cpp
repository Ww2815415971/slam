#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>
#include <chrono>
using namespace std;
using namespace cv;
void find_feature_matches(const Mat &img_1, const Mat &img_2,
std::vector<KeyPoint> &keypoints_1,
std::vector<KeyPoint> &keypoints_2,
std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p,const Mat &K);

typedef vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose 
);
void bundleAdjustmentGassNewton(
    const VecVector2d &points_3d,
    const VecVector3d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose
);
int main()
{
    Mat img1 = imread("../images/1.png",CV_LOAD_IMAGE_COLOR);
    Mat img2 = imread("../images/2.png",CV_LOAD_IMAGE_COLOR);
    assert(img1.data && img2.data && " Can not load images!");

    vector<KeyPoint> keypoints_1,keypoints_2;  // 关键点 图像中的特征点
    vector<DMatch>matches; // 匹配的结果
    find_feature_matches(img1,img2,keypoints_1,keypoints_2,matches);
}
void find_feature_matches(const Mat &img_1, const Mat &img_2,std::vector<KeyPoint> &keypoints_1,
std::vector<KeyPoint> &keypoints_2,
std::vector<DMatch> &matches)
{
    Mat descriptors_1,descriptors_2;

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);
    descriptor->compute(img_1,keypoints_1,descriptors_1);
    descriptor->compute(img_2,keypoints_2,descriptors_2);
    vector<DMatch> match;
    matcher->match(descriptors_1,descriptors_2,match);

    double min_dist = 10000,max_dist = 0;

    for(int i = 0; i < descriptors_1.rows; i++)
    {
        if(match[i].distance < min_dist) min_dist = match[i].distance;
        if(match[i].distance > max_dist) max_dist = match[i].distance;

    }
    std::cout <<"Max dist " << max_dist<<endl;
    std::cout << "Min dist " << min_dist<< endl;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if(match[i].distance < max(2 * min_dist,30.0))
        {
            matches.push_back(match[i]);
        }
    }
    std::cout << "共匹配" << matches.size() << endl;
    
}

Point2d pixel2cam(const Point2d &p,const Mat &K)
{
    return Point2d
    (
        (p.x - K.at<double>(0,2))/ K.at<double>(0,0),
        (p.y- K.at<double>(1,2)) / K.at<double>(1,1)
    );
}
void bundleAdjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose)
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double> (0, 0);
    double fy = K.at<double> (1, 1);
    double cx = K.at<double> (0, 2);
    double cy = K.at<double> (1, 2);

    for(int iter = 0; iter < iterations; iter ++)
    {
        // 海森矩阵
        Eigen::Matrix<double,6,6> H = Eigen::Matrix <double ,6, 6>::Zero();
        //误差矩阵
        Vector6d b = Vector6d::Zero();

        cost = 0;
        //遍历一堆三维点
        for (int i = 0; i < points_3d.size(); i++)
        {
            // 重投影误差法
            Eigen::Vector3d pc = pose * points_3d [i];
            // 将三维点与相机位姿相乘，得到在相机坐标系下的投影
            // 获取深度值
            double inv_z = 1.0 / pc[2];
            //计算雅可比矩阵使用 pc指的是
            double inv_z2 = inv_z * inv_z;
            //内参矩阵和
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx,fy * pc[1] / pc[2] +cy);

            Eigen::Vector2d e = points_2d[i] - proj;

            cost += e.squaredNorm(); // 求平方
            Eigen::Matrix<double,2,6> J;
            J << -fx *inv_z,
            0,
            fx * pc[0] *inv_z2,
            fx * pc[0] * pc[1] * inv_z2,
            -fx - fx * pc[0] * pc[0] * inv_z2,
            fx *pc[1] * inv_z,
            0,
            -fy * inv_z,
            fy * pc[1] * inv_z2,
            fy + fy * pc[1] * pc[1] *inv_z2,
            -fy * pc[0] * pc[1] *inv_z2,
            -fy * pc[0] *inv_z;

            H += J.transpose() * J;
            b += -J.transpose() *e;

        }
        Vector6d dx;
        // 求海森矩阵的更新方向
        dx = H.ldlt().solve(b);
        // 通过LDLT分解对称矩阵分解 求解线性方程组 H *dx  = b
        // b 为梯度或误差向量
        // dx 是求解得到的最优更新方向

        if(isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
        }
    }
}
