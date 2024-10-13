#include <iostream>
#include <chrono>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
int main(int argc,char ** argv)
{
   cv::Mat image;
   image = cv :: imread("/home/steven/slam/slambook/ch5/5.1/ubuntu.png");
   // 判断是否顺利读取文件
   if (image.data == nullptr)
   {
    cerr << "文件" << argv[1] << "不存在" << endl;
    return 0;
   }
   // 顺利读取之后 输出基本信息
   cout << "图像宽为" << image.cols << ",高为" << image.rows << "，通道数为" << image.channels() << endl;
   cv::imshow("image",image);
   cv:: waitKey(0); // 暂停程序，等待按键输入
   // 判断image的类型
   if(image.type() != CV_8UC1 && image.type() != CV_8UC3)
   // 图像类型不符合要求
   {
    cout << "请输入一张彩色图或灰度图" << endl;
    return 0;
   }
   // 遍历图像
   chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
   for (size_t y = 0; y < image.rows; y++)
   {
    unsigned char * row_ptr = image.ptr<unsigned char>(y); // 第y行的头指针
    for(size_t x = 0; x<image.cols; x++)
    {
        unsigned char *data_ptr = &row_ptr[x * image.channels()];// 由于是一维存储的，所以不能通过二维数组来访问像素值
        for(int c = 0; c != image.channels(); c++)
        {
            unsigned char data = data_ptr[c]; //为第 c 个通道的值
        }
    }
   }
chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
chrono::duration<double> time_used=  chrono::duration_cast < chrono::duration < double >> (t2-t1);
cout << "遍历图像用时：" << time_used.count() << "秒 "<<endl;

cv::Mat image_another = image;
// 修改image_anther导致image发生变化
image_another(cv::Rect(0,0,100,100)).setTo(0);
cv::imshow("image",image);
cv::waitKey(0);

//使用clone 函数拷贝数据
cv::Mat image_clone = image.clone();
image_clone(cv::Rect(0,0,100,100)).setTo(255);
cv::imshow("image",image);
cv::imshow("image_clone",image_clone);
cv::waitKey(0);
cv::destroyAllWindows();
return 0;

}