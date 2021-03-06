// 该文件将打开你电脑的双目摄像头，并将图像传递给ORB-SLAM3进行定位
 
// opencv
#include <opencv2/opencv.hpp>
 
// ORB-SLAM的系统接口
#include<System.h>
#include <string>
#include <chrono>   // 用于创建时间戳
#include <iostream>
 
using namespace std;
 
// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "./my2cam.yaml";
string vocFile = "../Vocabulary/ORBvoc.txt";
string videoFile = "./video.avi";

 
int main(int argc, char **argv) {
    ///////////////// 系统初始化 /////////////////
    // 获取相机图像
    cv::VideoCapture cap(videoFile);    
    // 分辨率设为640x480(双目为1280*480)
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);;//设置采集视频的宽度
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//设置采集视频的高度
    long frameToStart = 60;//给双目三秒钟的时间初始化
    cap.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
    cv::Rect Lrect(0,0, 1280/2,480);//左ROI
    cv::Rect Rrect(1280 / 2, 0, 1280/2, 480); //右ROI
    //读取畸变矫正参数
    cv::FileStorage fsSettings(parameterFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: 参数路径错误！" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: 参数不完整！" << endl;
        return -1;
    }
    cout<<"相机内参读取完成！"<<endl;
    // 计算畸变矫正参数
    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    cout<<"畸变矫正参数计算完成！"<<endl;

    // 声明 ORB-SLAM3 双目系统
    ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::STEREO, true);
    // 记录系统时间
    auto start = chrono::system_clock::now();
    // 逐帧处理视频
    while (cap.isOpened()) {
        cv::Mat frameTotal,imLeftRect, imRightRect;
        cap >> frameTotal;   // 读取相机数据
        //图像畸变矫正
        cv::remap(frameTotal(Lrect),imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(frameTotal(Rrect),imRightRect,M1r,M2r,cv::INTER_LINEAR);
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackStereo(imLeftRect,imRightRect,double(timestamp.count())/1000.0);
        cv::waitKey(30);
    }
    SLAM.Shutdown();
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    return 0;
}
