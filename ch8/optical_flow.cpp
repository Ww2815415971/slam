#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>


using namespace std;
using namespace cv;
string file_1 = "./img/LK1.png";
string file_2 = "./img/LK2.png";

//光流和接口

class OpticalFlowTracker
{
public:
    OpticalFlowTracker(
        const Mat &img_1,
        const Mat &img_2,
        const vector<KeyPoint> &kp1_,
        vector<KeyPoint> &kp2_,
        vector<bool> &success_,
        bool inverse_= true,bool has_initial_= false):
        img1(img_1),img2(img_2),kp1(kp1_),kp2(kp2_),
        has_initial(has_initial_) {}

        void calculateOpticalFlow(const Range &range );
    
private:
        const Mat &img1;
        const Mat &img2;
        const vector <KeyPoint>&kp1;
        const vector <KeyPoint>&kp2;
        bool inverse=true;
        bool has_initial = false;

};

/**
 * single level optical flow
 * @param [in] img1 首图片
 * @param [in] img2 第二个图片
 * @param [in] kp1 第一个图片关键点
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * 
 * @param [out] success 成功跟踪的关键点的数目
 * @param [in] inverse use inverse formulation?
 */

 void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> & kp2,
    vector<bool> &success,
    bool inverse = false,
    bool has_initial_guess = false

);
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultivel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return the interpolated value of this pixel
 */

 inline float GetPixelValue(const cv::Mat &img,float x,float y)
 {
    if(x <0) x = 0;
    if(y <0) y = 0;
    if (x >=img.cols - 1) x = img.cols-2;
    if(y>= img.rows-1) y = img.rows-2;

    float xx = x- floor(x);
    float yy = y - floor(y);
    int x_a1 = std::min(img.cols - 1, int(x)+1);
    int y_a1 = std::min(img.rows-1,int (y)+1);
    return (1 - xx)* (1-yy) * img.at<uchar>(y,x)
 }