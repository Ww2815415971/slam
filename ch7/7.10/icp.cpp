#include<iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>

using namespace std;
using namespace cv;
void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches
);
Point2d pixel2cam(
    const Point2d &p, const Mat &K);
void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,Mat &R, Mat&t);
//

void bundleAdjustment(
    const vector<Point3f> &points_3d,
    const vector<Point3f> &points_2d,
    Mat &R, Mat &t
);// 由于获取的数据为相机和激光雷达的数据，
//仍保留二维属性，获取的点云数据不完全，所以还是需要转化成2d的数据去进行优化


//继承g2o的基础顶点类       6自由度的位姿和李代数
class VertexPose :public g2o::BaseVertex<6,Sophus::SE3d> {
    
    public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;//提高效率

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();//初始化误差为0
    }

    virtual void oplusImpl(const double *update) override {
        Eigen::Matrix<double,6,1> update_eigen;//更新矩阵
        update_eigen << update[0],update[1],update[2],update[3],update[4],update[5]; // 填充更新矩阵
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;//更新估计值

    }
    virtual bool read(istream &in) override {}
    virtual bool write(ostream &out) const override {}


};
   
    // g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3,Eigen::Vector3d,VertexPose>{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point)
    {}   // 构造函数
        virtual void computeError() override{
            
            // 获取第一个顶点的指针
            const VertexPose * pose = static_cast<const VertexPose *> (_vertices[0]); 
            
            // 通过_measurement观测值与位姿的估计值求的误差
            _error = _measurement - pose->estimate() * _point;
        }
    virtual void linearizeOplus() override{
        VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);//用安全的方式构造派生类指针
        Sophus::SE3d T = pose->estimate();// 获取位姿的估计值
        Eigen::Vector3d xyz_trans = T * _point;// 根据位姿的估计值和点的坐标去得到变换后的坐标
        // 获取一个负的单位矩阵
        _jacobianOplusXi.block<3,3>(0,0) = - Eigen::Matrix3d::Identity();
        // 将估计的左边转换成李代数存入矩阵中 
        _jacobianOplusXi.block<3,3>(0,3) = Sophus::SO3d::hat(xyz_trans);

    }
    // 为了后续的可扩展性，这里的read和write不实现
    bool read(istream &in) {}
    bool write(ostream &out) const {}


    protected:
    Eigen::Vector3d _point;
};
int main()
{
   Mat img1 = imread("../images/1.png",CV_LOAD_IMAGE_COLOR);
   Mat img2 = imread("../images/2.png",CV_LOAD_IMAGE_COLOR);

   vector<KeyPoint> keypoint_1,keypoint_2;
   vector<DMatch> matches;
   find_feature_matches(img1,img2,keypoint_1,keypoint_2,matches);
   Mat depth1 =imread("../images/1_depth.png",CV_LOAD_IMAGE_UNCHANGED);
   Mat depth2 =imread("../images/2_depth.png",CV_LOAD_IMAGE_UNCHANGED); // 16位无符号数
   Mat K = (Mat_<double>(3,3) << 520.9,0,325.1,0,521.0,249.7,0,0,1);
    vector<Point3f> pts1,pts2;
    //  从匹配子中获取深度值
    for(DMatch m:matches)
    {
        ushort d1 = depth1.ptr<unsigned short>(int (keypoint_1[m.queryIdx].pt.y))[int(keypoint_1[m.queryIdx].pt.x)];
        ushort d2 = depth2.ptr<unsigned short>(int (keypoint_2[m.trainIdx].pt.y))[int(keypoint_2[m.trainIdx].pt.x)];       
        if(d1 == 0 || d2 == 0)
        continue;
        Point2d p1 = pixel2cam(keypoint_1[m.queryIdx].pt,K);
        Point2d p2 = pixel2cam(keypoint_2[m.trainIdx].pt,K);
        float dd1 = float(d1)/5000.0;
        float dd2 = float(d2)/5000.0;
        pts1.push_back(Point3f(p1.x *dd1,p1.y *dd1,dd1));
        pts2.push_back(Point3f(p2.x *dd2, p2.y *dd2,dd2));

    }
    cout << "3d-3d pairs:" << pts1.size() << endl;
    Mat R,t;
    pose_estimation_3d3d(pts1,pts2,R,t);

    cout << "ICP via SVD results:"<< endl;
    cout << "R = "<< R << endl;
    cout << "t = "<< t << endl;
    cout << "R_inv ="<< R.t()<< endl;
    cout << "calling bundle adjustment" <<endl;
    bundleAdjustment(pts1,pts2,R,t);

    // 验证BA结果
    for (int i = 0; i< 5; i ++)
    {
        cout << "p1 = " << pts1[i] << endl;
        cout << "p2 = " << pts2[i] << endl;
        cout << "(R*p2+t) = " << 
        R * (Mat_<double> (3,1) << pts2[i].x,pts2[i].y,pts2[i].z) + t
        << endl;
        cout << endl;
    }    
}
void find_feature_matches(const Mat &img1,const Mat &img2,
vector<KeyPoint> &keypoints1,
vector<KeyPoint> &keypoints2,
vector<DMatch> &matches)
{
    // 构建描述子
    Mat descriptors1,descriptors2;
    // 创建一个检测子
    Ptr<FeatureDetector> detector = ORB::create(); 
    // 创建一个描述子选择子
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // 创建一个描述字匹配子
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> match;
    detector->detect(img1,keypoints1);
    detector->detect(img2,keypoints2);
    
    descriptor->compute(img1,keypoints1,descriptors1);
    descriptor->compute(img2,keypoints2,descriptors2);
    matcher->match(descriptors1,descriptors2,match);

    double min_dist = 10000,max_dist= 0;
    for(int i = 0; i < descriptors1.rows;i++)
    {
        double dist = match[i].distance;
        if( dist < min_dist) min_dist = dist;  
        if(dist > max_dist) max_dist = dist;
    }
    printf("Max dist :%f\n",max_dist);
    printf("Min dist :%f\n",min_dist);

    for(int i = 0 ; i < descriptors1.rows; i++)
    {
        if(match[i].distance <= max(2 * min_dist,30.0))
        {
            matches.push_back(match[i]);
           
        }
     }
    printf("共匹配点数：%d\n",matches.size());

}
// 根据像素坐标和相机内参去获得相机坐标
Point2d pixel2cam(const Point2d &p ,const Mat &K)
{
    return Point2d(
        (p.x -K.at<double>(0,2))/K.at<double>(0,0),
        (p.y - K.at<double>(1,2))/K.at<double>(1,1)
        );
}

void pose_estimation_3d3d(const vector<Point3f> &pts1,
const vector<Point3f> &pts2,Mat &R, Mat &t )
{
    Point3f p1,p2; // 求质心
    int N = pts1.size();
    
    // 通过去平均值去除重心，从而减少平移对估计的影响
    for(int i = 0; i <  N; i ++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    // 开一个数组去存储去重心后的点
    vector<Point3f> q1(N),q2(N);
    for(int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // 计算 q1 * q2 ^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero(); //创建一个3*3的零矩阵
    for(int i = 0; i < N; i++)
    {    
        // 将去除重心的点转成向量再相乘累加
        W += Eigen::Vector3d(q1[i].x,q1[i].y,q1[i].z) * Eigen::Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose(); 
    }
    cout << "W= "<< W << endl;

    // 对累加的矩阵进行奇异值分解
    // 获得奇异值矩阵
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
    // 从奇异值矩阵里面获取U和V
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    cout << "U=" <<U << endl;
    cout << "V=" << V << endl;
    
    // 根据SVD分解的结果得到U和V得到旋转矩阵
    Eigen::Matrix3d R_ = U *(V.transpose());
    // 判断旋转矩阵是否为的值是否为正的，因为旋转矩阵本身就是一个正交矩阵，所以行列式为1
    if(R_.determinant() < 0)
    {
        R_ = -R_;
    }
    Eigen::Vector3d t_ =Eigen::Vector3d(p1.x,p1.y,p1.z) -R_ * Eigen::Vector3d(p2.x,p2.y,p2.z);
   
    // 转换cv::Mat
    R = (Mat_<double> (3,3) <<
     R_(0, 0),R_(0, 1),R_(0, 2),
     R_(1, 0),R_(1, 1),R_(1, 2),
     R_(2, 0),R_(2, 1),R_(2, 2));   
     t = (Mat_<double> (3,1) << t_(0,0),t_(1,0),t_(2,0));
}
void bundleAdjustment(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R,Mat &t
)
{
    // 初始化g2o
    typedef g2o::BlockSolverX BlockSolver;
    //构建一个线性求解器
    typedef g2o::LinearSolverDense<BlockSolver::PoseMatrixType> LinearSolverType;

    // 创建迭代器，传入一个利用std::make_unique创建一个智能指针管理线性求解器
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolver>(std::make_unique<LinearSolverType>())
    );
    // 好处是可以自动释放内存
    
    //将迭代器和传入图模型中
    g2o::SparseOptimizer optimizer;// 图模型
    optimizer.setAlgorithm(solver);// 将求解器设置到g2o优化器中
    optimizer.setVerbose(true);// 打开调试输出

    // 定点
    VertexPose *pose = new VertexPose();
    pose->setId(0);
    pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(pose);



    // 边

    for(size_t i = 0; i < pts1.size();i++)
    {
        // 创建一个边节点，将第二个点的3d坐标传入边结点中
        EdgeProjectXYZRGBDPoseOnly * edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x,pts2[i].y,pts2[i].z)
        );
        // 这个是将edge与顶点的序号和顶点变量的变量名进行绑定
        edge->setVertex(0,pose);
        // 设置第一个点的3d坐标作为观测值，让第二个点的3d坐标经过位姿变换之后和这个点误差最小
        edge->setMeasurement(Eigen::Vector3d
        (pts1[i].x,pts1[i].y,pts1[1].z));
        // Information 矩阵用于表示边的置信度或测量的精度。
        // 单位矩阵表示各个维度之间没有相关性，并且所有维度的置信度相同。
        edge->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(edge); 

    }
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();// 初始化优化器
    optimizer.optimize(10);// 设置优化次数
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 -t1);
    cout << "optimizer costs time:"<< time_used.count() << "seconds." << endl;

    cout <<endl<<"after optimization:"<< endl;
    cout << "T=\n" << pose->estimate().matrix()<< endl;

    // 转成cv::Mat 从李代数中获取平移和旋转矩阵
    Eigen::Matrix3d R_ =pose->estimate().rotationMatrix();
    Eigen::Vector3d t_ = pose->estimate().translation();

    R = (Mat_<double>(3,3) << 
    R_(0,0),R_(0,1),R_(0,2),
    R_(1,0),R_(1,1),R_(1,2),
    R_(2,0),R_(2,1),R_(2,2));

    t = (Mat_<double>(3,1)<< t_(0,0),t_(1,0),t_(2,0));

}



