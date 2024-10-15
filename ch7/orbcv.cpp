#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int main()
{
    Mat img_1 = imread("/home/steven/slam/slambook/ch7/images/1.png",IMREAD_COLOR);
    Mat img_2 = imread("/home/steven/slam/slambook/ch7/images/2.png",IMREAD_COLOR);//以彩色形式读取图片
    assert(img_1.data != nullptr&& img_2.data != nullptr);//断言去判断是否读取到图片
    // 初始化
    std ::vector<KeyPoint> keypoint_1,keypoint_2;// 定义角点
    Mat descriptors_1,descriptors_2;// 定义描述子
    Ptr<FeatureDetector> detector = ORB::create();//创建特征提取
    Ptr<DescriptorExtractor> descriptor = ORB::create();//创建特征提取的描述子
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");//暴力汉明吗匹配
    
    // 检测角点
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 开始时间
    detector->detect(img_1,keypoint_1); // 根据图片提取角点，
    detector->detect(img_2,keypoint_2); 

    // 根据角点位置计算BRIFT描述子
    descriptor->compute(img_1,keypoint_1,descriptors_1); //去计算得到描述子
    descriptor->compute(img_2,keypoint_2,descriptors_2); 
    chrono ::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono ::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "extact keypoint used" << time_used.count()<< endl;
    
    Mat outimg1;
    // 标记出角点
    drawKeypoints(img_1,keypoint_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("ORB features",outimg1);

    // 对两幅图片中的BRIEF描述字进行匹配，使用Hamming距离
    vector<DMatch>matches; // 进行匹配
    t1 = chrono::steady_clock::now();
    matcher->match (descriptors_1,descriptors_2,matches);//利用描述子进行匹配，保存到matches中
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout <<"match ORB cost = "<<time_used.count()<< endl;

    // 对匹配点进行遍历筛选
    auto min_max = minmax_element(matches.begin(),matches.end(),[](const DMatch &m1,const DMatch &m2)
    {
        return m1.distance <m2.distance;
    }
    );
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    printf("-- Max dist : %f \n",max_dist);
    printf("-- Min dist : %f \n",min_dist);
    std::vector<DMatch> good_matches;
    for(int i = 0; i < descriptors_1.rows; i++)
    {
        if(matches[i].distance <= max(2 * min_dist,30.0))
        {
            good_matches.push_back(matches[i]);
        }

    }
    // 获取匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,matches,img_match);
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,good_matches,img_goodmatch);
    imshow("all matches",img_match);
    imshow("good matches",img_goodmatch);
    waitKey(0);
    return 0;

}
