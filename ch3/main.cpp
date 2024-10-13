#include <epoxy/gl.h>     
#include <GL/gl.h>        // 随后添加 OpenGL 头文件
#include <pangolin/pangolin.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <unistd.h>
#include <fstream>
#include <iostream>



using namespace std;
using namespace Eigen;

//  轨迹文件
string trajectory_file="/home/steven/slam/slambook/ch3/trajectory.txt";

void DrawTrajectory(vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>>);//定义显示轨迹的函数

int main(int argc,char ** argv)
{
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;//创建对应的动态数组，以固定内存分配方式来实现
    ifstream fin(trajectory_file);
    if(!fin){
        cout << "cannot find trajectory file at " << trajectory_file<<endl;
        return 1;    
    }
    // 读入时间，位置，和摄像头的位姿
    while(!fin.eof()){
        double time,tx,ty,tz,qx,qy,qz,qw;
        fin >> time >>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Isometry3d Twr(Quaternion(qw,qx,qy,qz));//将位姿转换成变换矩阵
        Twr.pretranslate(Vector3d(tx,ty,tz));//先将向量转换成 4*4 的变换矩阵，再与 Twr 相乘
        poses.push_back(Twr);
    }
    cout << "read total "<<poses.size()<< "pose entries" <<endl;
    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}
void DrawTrajectory(vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>>poses){
    //创建一个pangolin页面
    pangolin::CreateWindowAndBind("Trajectory Viewer",1024,768);
    // 会让图像显示更加细腻颜色混合度更高
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    pangolin :: OpenGlRenderState s_cam(
        pangolin :: ProjectionMatrix(1024,768, // 屏幕宽度和高度
        500, 500, // 焦距
        512,389,  
        0.1,1000),// 模拟相机内参
        pangolin:: ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0) // 保存相机在世界坐标系下的一个位置信息 
    );
    pangolin::View &d_cam =pangolin ::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam)); // 创建视图
    
//  读取坐标信息进行显示
    while(pangolin:: ShouldQuit()==false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++)
        {   
        // 画出坐标轴并给不同坐标轴不同的颜色表示
            Vector3d Ow=poses[i].translation();
            Vector3d Xw=poses[i] * (0.1 * Vector3d(1,0,0));
            Vector3d Yw=poses[i] * (0.1 * Vector3d(0,1,0));
            Vector3d Zw=poses[i] * (0.1 * Vector3d(0,0,1));
            glBegin(GL_LINES);
            glColor3f(1.0,0.0,0.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Xw[0],Xw[1],Xw[2]);
            glColor3f(0.0,1.0,0.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Yw[0],Yw[1],Yw[2]);
            glColor3f(0.0,0.0,1.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Zw[0],Zw[1],Zw[2]);
            glEnd();
            

        }
        // 画出连线
        for (size_t i = 0; i < poses.size(); i++)
        {
            glColor3f(0.0,0.0,0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i],p2 = poses[i+1];
            glVertex3d(p1.translation()[0],p1.translation()[1],p1.translation()[2]);
            glVertex3d(p2.translation()[0],p2.translation()[1],p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);
        
    }
}