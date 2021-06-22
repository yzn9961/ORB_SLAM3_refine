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
string Left_videoFile = "./left.avi";
string Right_videoFile = "./right.avi";
 
int main(int argc, char **argv) {
    ///////////////// 系统初始化 /////////////////
    // 获取相机图像
    cv::VideoCapture capl(Left_videoFile);
    cv::VideoCapture capr(Right_videoFile);    
    // 分辨率设为640x480(双目为1280*480)
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);;//设置采集视频的宽度
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//设置采集视频的高度
    long frameToStart = 60;//给双目三秒钟的时间初始化
    capl.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
    capr.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
    // 声明 ORB-SLAM3 双目系统
    ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::STEREO, true);
    // 记录系统时间
    auto start = chrono::system_clock::now();
    // 逐帧处理视频
    while (capl.isOpened() && capr.isOpened()) {
        cv::Mat imLeft, imRight;
        capl >> imLeft;   // 读取相机数据
	capr >> imRight;
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackStereo(imLeft,imRight,double(timestamp.count())/1000.0);
        cv::waitKey(30);
    }
    SLAM.Shutdown();
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    return 0;
}
