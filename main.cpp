#include "constparas.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>

int main()
{
    //读入图像
//    cv::Mat src = cv::imread("/home/liu/图片/14.jpg");
    cv::VideoCapture inputVideo;
    inputVideo.open(0);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    //读入相机标定的参数
    cv::Mat cameraMatrix;
    cv::Mat cameraDistCoeffs;
    cv::FileStorage fs("/home/liu/图片/ccalib/calib.xml", cv::FileStorage::READ);
    fs["Intrinsic"] >> cameraMatrix;
    fs["Distortion"] >> cameraDistCoeffs;

    while (inputVideo.grab())
    {
        //time
        //double startTime = (double)cv::getTickCount();
        
        cv::Mat src;
        inputVideo.retrieve(src);
        
        //time
        //double runTimeSub = ((double)cv::getTickCount() - startTime) / cv::getTickFrequency();
        //std::cout <<"1: "<<runTimeSub << std::endl;

        //检测标记
        cv::aruco::detectMarkers(src, dictionary, markerCorners, markerIds);
        
        //相机姿态估计
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, QRCODEUNIT, cameraMatrix, cameraDistCoeffs, rvecs, tvecs);

        if(rvecs.size() > 0)
        {
            //获取二维码相对于相机的位置
/*            cv::Mat rotation;
            cv::Rodrigues(rvecs[0], rotation);
            cv::Mat cameraZero = - rotation.inv() * cv::Mat(tvecs[0]);

            std::cout<<"cameraZero: "<<cameraZero<<std::endl;
*/
            //test
//            std::cout<<"rvecs[0]: "<<rvecs[0];
            std::cout<<" tvecs[0]: "<<tvecs[0]<<std::endl;

        }

        //test
        for(unsigned int i=0; i<markerIds.size(); i++)
            cv::aruco::drawAxis(src, cameraMatrix, cameraDistCoeffs, rvecs[i], tvecs[i], QRCODEUNIT);
        cv::namedWindow("test",0);
        cv::imshow("test",src);

        char key = (char) cv::waitKey(1);
        if (key == 27)
            break;
    }


    return 0;
}

