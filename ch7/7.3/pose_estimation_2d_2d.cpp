#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

// 只去定义一个函数去容纳特征点的信息，以及对应描述子的数组
void find_feature_matches(
    const Mat &img_1,const Mat &img_2,//图像1 ，图像2
    std::vector<KeyPoint> &keypoints_1,//特征点1
    std::vector<KeyPoint> &keypoints_2,//特征点2
    std::vector<DMatch> &matches // 描述子
  );

void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,// 关键帧
    std::vector<KeyPoint> keypoints_2,//关键帧
    std::vector<DMatch> matches,// 
    Mat &R,Mat &t// 旋转矩阵和平移矩阵得到的变换矩阵
    );

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p,const Mat & K );
int main()
{
    Mat img1 = imread("../images/1.png",CV_LOAD_IMAGE_COLOR);
    Mat img2 = imread("../images/2.png",CV_LOAD_IMAGE_COLOR);
    assert(img1.data && img2.data && " Can not load images!");

    vector<KeyPoint> keypoints_1,keypoints_2;
    vector<DMatch>matches;
    find_feature_matches(img1,img2,keypoints_1,keypoints_2,matches);
    cout<<"一共找到了"<< matches.size()<<"组匹配点"<<endl;
    // 估计两张图像间运动
    Mat R,t; 
    // R 是旋转 t 是平移

    pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);
   
    // 验证 E = t^R*scale
    // 将一个向量<tx,ty,tz>转为反对称矩阵
    // -t.at<double>(n,0), 向量的第n+1个元素 
    Mat t_x = (Mat_<double>(3,3) << 0 , -t.at<double>(2,0),t.at<double>(1,0),
    t.at<double>(2,0),0,-t.at<double>(0,0),
    -t.at<double>(1,0),t.at<double>(0,0),0);
    
    cout << "t^R=" <<endl <<t_x * R <<endl;

// 验证对极约束
    Mat K = (Mat_<double>(3,3) << 520.9 ,0, 325.1,
                                 0,521.0,249.7,
                                0,0,1);
// 利用Mat_<double>(3,3)创建一个3*3的矩阵，并通过 << 来赋值
    for(DMatch m:matches)// 遍历匹配子
    {   

        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt,K);
        // 定义一个二维点 pt1 
        //keypoints_1[m.queryIdx].pt 从第一张图像匹配的特征点索引，获取特征点
        //.pt表示 特征点的像素坐标
        // K 是相机的内参矩阵
        // m.queryIdx 指的是第一张图片的索引
        Mat y1 = (Mat_<double> (3,1) << pt1.x,pt1.y,1) ;
        // 创建一个三行一列的矩阵，并把第二张图片的特征点像素坐标值的转成齐次坐标
        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt,K);
        Mat y2 = (Mat_<double> (3,1) << pt2.x,pt2.y,1);
        Mat d = y2.t() * t_x * R *y1;
            // 验证 x2^t * t^ R * x1 = 0
            // R 是旋转，t是平移
              // y2 的转置 * 平移向量的反对称矩阵 * 旋转矩阵 * 
            cout << "epipolar constraint = "<< d << endl;
            // 输出结果
    }
    return 0;
}
void find_feature_matches(const Mat &img_1,const Mat & img_2, 
std::vector<KeyPoint> &keypoints_1,
std::vector<KeyPoint> &keypoints_2, std::vector<DMatch> & matches)
{
    // 初始化
    // Mat与Ptr  
    // Mat 表示图像信息
    // Ptr 表示智能指针类用来管理内存防止内存问题
    // 描述子
    Mat descriptors_1 ,descriptors_2;
    //创建ORB特征检测器
    Ptr<FeatureDetector>  detector = ORB::create();
    // 创建ORB特征提取器 从匹配器中去提取描述子
    Ptr<DescriptorExtractor> descriptor = ORB::create(); 
    // 匹配方式 使用暴力匹配汉明距离去创建匹配器
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher :: create("BruteForce-Hamming");
   



   //检测角点位置
    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

   // 根据角点位置计算BRIFT 描述子
   descriptor->compute(img_1,keypoints_1,descriptors_1);
   descriptor->compute(img_2,keypoints_2,descriptors_2);

   // 对两幅图片的BRIEF描述子进行匹配
   vector<DMatch> match;
   matcher->match(descriptors_1,descriptors_2,match);

   // 匹配点对筛选
    //类似与比大小，找到int中的最大值和最小值进行比较找到里面的最值
   double min_dist = 10000,max_dist = 0;
   
   for(int i = 0; i < descriptors_1.rows; i++)
   {
    double dist = match[i].distance;
    if(dist <min_dist) min_dist=dist;
    if(dist >max_dist) max_dist= dist;
   }

   printf("-- Max dist:%f\n",max_dist);
   printf("-- Min dist:%f\n", min_dist);

   //当描述子之间的距离大于二倍的最小距离,认为匹配有误，最小值小于30时
   for(int i = 0;i <descriptors_1.rows;i ++)
   {
    if(match[i].distance <= max(2*min_dist,30.0))
    {
        matches.push_back(match[i]);
    }
   }
}

//像素坐标转相机归一化坐标
                    //2d坐标             相机的外参矩阵
Point2d pixel2cam(const Point2d &p, const Mat &K)
{       
    Point2d p1(p.x - K.at<double>(0,2) / K.at<double>(0,0),
    (p.y - K.at<double>(1,2) / K.at<double>(1,1)));
    
    // px-cx/fx
    // py-cy/fy

    /*
        {
        fx 0 cx
    K=  0 fy cy  
        0 0 1  
        }
 

    */
    return p1;
}

void pose_estimation_2d2d(std::vector<KeyPoint>keypoints_1,
std::vector<KeyPoint>keypoints_2,
std::vector<DMatch> matches,
Mat &R, Mat &t)
{
    // 相机内参
    Mat K = (Mat_<double>(3,3) << 520.9 ,0, 325.1,
                                 0,521.0,249.7,
                                0,0,1);
    //将匹配点转换成 vector<Point2f>
    vector<Point2f> points1;
    vector<Point2f> points2;
    for (int  i = 0; i < (int)matches.size(); i++)
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);

    }
    // -- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix =findFundamentalMat(points1,points2,CV_FM_8POINT);
    cout << "fundamental_matrix is "<< endl <<fundamental_matrix<<endl;


    // -- 计算本质矩阵
    Point2d principal_point(325.1,249.7); //相机光心
    double focal_length = 521; // 焦距 TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1,points2,focal_length,principal_point);
    cout <<"essential_matrix is "<< endl <<essential_matrix<<endl;
    

    // == 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography(points1,points1,RANSAC,3);
    cout <<"homography_matrix is "<< endl<<homography_matrix<<endl;

    //从本质矩阵中恢复旋转和平移信息
    // 此函数只在Opencv3中提供
    recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
     cout << "R is " << endl << R << endl;
     cout << "t is " << endl << t << endl;

}


