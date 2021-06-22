// 该文件将打开你电脑的双目摄像头，并将图像传递给ORB-SLAM3进行定位
 
// opencv
#include <opencv2/opencv.hpp>
 
// ORB-SLAM的系统接口
#include<System.h>
#include <string>
#include <chrono>   // for time stamp
#include <iostream>
 
using namespace std;
 
// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "./my2cam.yaml";
string vocFile = "../Vocabulary/ORBvoc.txt";
 
int main(int argc, char **argv) {
    ///////////////// 系统初始化 /////////////////
    // 获取相机图像
    cv::VideoCapture cap(2);    // change to 1 if you want to use USB camera.
    // 分辨率设为640x480(双目为1280*480)
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);;//设置采集视频的宽度
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//设置采集视频的高度
    long frameToStart = 90;//给双目三秒钟的时间初始化
    cap.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
    cv::Rect Lrect(0,0, 1280/2,480);//左ROI
    cv::Rect Rrect(1280 / 2, 0, 1280/2, 480); //右ROI
    //读取畸变矫正参数
    cv::FileStorage fsSettings(parameterFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l; //手动传入R_l
    /*R_l = (cv::Mat_<double>(3, 3) << 9.9998763455599071e-01, 1.1486063626280848e-03,
                        -4.8385368178863117e-03, -1.1513689342545705e-03,
                        9.9999917574594432e-01, -5.6820507631147495e-04,
                        4.8378801857367881e-03, 5.7376899118277886e-04,
                        9.9998813278181109e-01);*/
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
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }
    // 计算畸变矫正参数
    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    //std::cout<<R_l<<endl;
    // 声明 ORB-SLAM3 双目系统
    ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::STEREO, true);
    // 记录系统时间
    auto start = chrono::system_clock::now();
    while (1) {
        cv::Mat frameTotal,imLeftRect, imRightRect;
        cap >> frameTotal;   // 读取相机数据
        //图像畸变矫正
        cv::remap(frameTotal(Lrect),imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(frameTotal(Rrect),imRightRect,M1r,M2r,cv::INTER_LINEAR);
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        //将当前图像传输给SLAM系统
        //cv::imshow("img",frameTotal);
        //cv::waitKey(20);
        SLAM.TrackStereo(imLeftRect,imRightRect,double(timestamp.count())/1000.0);
        //SLAM.TrackMonocular(frame, double(timestamp.count())/1000.0);
    }

    return 0;
}
