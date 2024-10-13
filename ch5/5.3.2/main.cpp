#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
using namespace std;
string image_file = "/home/steven/slam/slambook/ch5/5.3.2/distorted.png";
int main (int argc, char ** argv)
// 实现去畸变部分的代码
{
    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
    cv :: Mat image = cv::imread(image_file,0);
    if (image.empty())
    {
        
        return -1;
    }
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);// 去畸变之后的图
    for (int  v = 0; v < rows; v++)
    {
        for (int u = 0; u < cols; u++)
        {
            double x = (u - cx )/fx, y = (v - cy )/fy;
            double r = sqrt(x * x + y * y);
            double x_disorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_disorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_disorted = fx * x_disorted + cx;
            double v_disorted = fy * y_disorted + cy;
            // 统一进行映射
            if (u_disorted >= 0  && v_disorted >= 0 && u_disorted <cols && v_disorted <rows)
            {
                image_undistort.at<uchar>(v,u) = image.at<uchar>((int)v_disorted,(int)u_disorted);

            }else
            {
                image_undistort.at<uchar>(v,u) = 0;
            }
        }
    }
    //printf("suibian");
    cv::imshow ("disorted",image);
    cv::imshow("undisorted",image_undistort);
    cv::waitKey();
    return 0;


}