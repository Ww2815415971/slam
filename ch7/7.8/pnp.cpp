#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
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
            break;
        }
        if(iter > 0 && cost >= lastCost)
        {
            cout << "cost: "<<cost <<", last cost:" << lastCost;
            break;
        }
        pose = Sophus::SE3d::exp(dx) *pose;
        lastCost = cost;

        cout << "iteration" << iter << "cost = " << std::setprecision(12) << cost<<endl;
        // 导数值为0；
        if(dx.norm() < 1e-6)
        {
            break;
        }
    }
        cout << "pose by g-n: \n" <<pose.matrix()<< endl;

    }
    class VertexPose : public g2o::BaseVertex<6 , Sophus::SE3d>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      // 提高效率
        //去进行在图优化时结点进行初始化，改为默认值单位矩阵，无平移无旋转
        virtual void setToOriginImpl()override {
            _estimate = Sophus::SE3d();
        }
        virtual void oplusImpl(const double *update) override
        {
            Eigen::Matrix<double,6,1> update_eigen;
            update_eigen << update[0],update[1],update[2],update[3],update[4],update[5];
           // 将更新数组转化为Eigen矩阵形式 
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
            // 将群映射到指数映射并去使用群乘法去进行更新，隐含了雅可比矩阵在里面
        }
        //读入和读出顶点的数据
        virtual bool read(istream &in) override{}

        virtual bool write(ostream &out) const override{}
    };

class EdgeProjection : public g2o::BaseUnaryEdge<2,Eigen::Vector2d,VertexPose>
{
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Eigen::Vector3d &pos,const Eigen::Matrix3d &K): _pos3d(pos),_K(K)
        {}

        virtual void computeError() override
        {
            const VertexPose *v =static_cast<VertexPose *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_pixel = _K *(T * _pos3d);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>(); 
        }
        virtual void linearizeOplus() override{
            const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_cam = T*_pos3d;
            double fx = _K(0,0);
            double fy = _K(1,1);
            double cx = _K(0,2);
            double cy = _K(1,2);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Z2 = Z * Z;
            _jacobianOplusXi
            << -fx/Z, 0, fx * X / Z2, fx * X *Y /Z2, -fx -fx * X * X /Z2,fx * Y /Z,
            0, -fy / Z, fy * Y /(Z *Z), fy + fy * Y * Y /Z2, -fy * X * Y / Z2, -fy * X /Z;

        }
        virtual bool read(istream &in) override {}
        virtual bool write(ostream &out) const override{}

    private:
     Eigen:: Vector3d _pos3d;
     Eigen::Matrix3d _K;
};

void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose)
{
        // 构建图优化 设定g2o

        // g2o 的pose is 6 landmark is 3 
        // 位姿是 6 维的包括相机的位置xyz 和Z: Yaw（绕 Z 轴旋转）。
        // Y: Pitch（绕 Y 轴旋转）。 当为+-90时 会导致万向锁，即丢失自由度
        // X: Roll（绕 X 轴旋转）   
        //和姿态 路标是 3 维的 xyz
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; 
        // 线性求解器
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // 梯度下降方法
        // config 配置求解器，初始化图模型，打开调试输出
        auto solver = new g2o::OptimizationAlgorithmGaussNewton(
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
            g2o::SparseOptimizer optimizer; // 图模型
            optimizer.setAlgorithm(solver); // 设置求解器
            optimizer.setVerbose(true);    // 打开调试输出

        // vertex init

        VertexPose *vertex_pose = new VertexPose();
        
        
}
