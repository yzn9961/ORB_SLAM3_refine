// 该文件将打开你电脑的摄像头，并将图像传递给ORB-SLAM2进行定位
 
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
string parameterFile = "./my1cam.yaml";
string vocFile = "../Vocabulary/ORBvoc.txt";
string videoFile = "./video.avi";
cv::Rect Lrect(0,0, 1280/2,480);//左ROI
 
int main(int argc, char **argv) {
 
    // 声明 ORB-SLAM3 系统
    ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::MONOCULAR, true);
 
    // 获取相机图像代码
    cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.
 
    // 分辨率设为640x480
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);;//设置采集视频的宽度
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//设置采集视频的高度
    long frameToStart = 90;
    cap.set(CV_CAP_PROP_POS_FRAMES, frameToStart);

    // 记录系统时间
    auto start = chrono::system_clock::now();
 
    while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        if ( frame.data == nullptr )break;
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame(Lrect), double(timestamp.count())/1000.0);
        cv::waitKey(30);
    }
    SLAM.Shutdown();
    return 0;
}
