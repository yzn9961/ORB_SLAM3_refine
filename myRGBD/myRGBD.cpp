#include<opencv2/opencv.hpp>
#include<System.h>
#include <string>
#include <chrono>   // 用于创建时间戳
#include <iostream>

using namespace std;
using namespace cv;
string Path_paramFile = "../Depthdataset/dorm/mydepth.yaml"; //相机参数
string Path_vocFile = "../Vocabulary/ORBvoc.txt";// 词袋位置
string Path_sequenceFolder = "../Depthdataset/mydesk";//  图像序列位置


int main(int argc, char**argv)
{
    // 读取RGB及深度图像路径
    vector<String> RGB_img_Paths;
    glob(Path_sequenceFolder+"/rgb",RGB_img_Paths);
    vector<String> Depth_img_Paths;
    glob(Path_sequenceFolder+"/depth",Depth_img_Paths);

    //检测是否成功读取数据集
    int nImages = Depth_img_Paths.size();

    if(RGB_img_Paths.empty())
    {
        cerr << endl << "未找到数据集！" << endl;
        return 1;
    }
    else if(Depth_img_Paths.size()!=RGB_img_Paths.size())
    {
        cerr << endl << "深度图像与RGB图像数量不同！" << endl;
        return 1;
    }
    //声明SLAM系统
    ORB_SLAM3::System SLAM(Path_vocFile, Path_paramFile, ORB_SLAM3::System::RGBD, true);
    cv::Mat imRGB, imD;
    // 记录系统时间
    auto start = chrono::system_clock::now();
    //主循环
    for(int n=0;n<nImages;n++)
    {
        //读取图像
        imRGB = imread(RGB_img_Paths[n],IMREAD_UNCHANGED);
        imD = imread(Depth_img_Paths[n],IMREAD_UNCHANGED);
        //创建时间戳
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        double tframe = double(timestamp.count())/1000.0;
         SLAM.TrackRGBD(imRGB,imD,tframe);
         cv::waitKey(30);
    }
    SLAM.Shutdown();
// Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   
    return 0;
}