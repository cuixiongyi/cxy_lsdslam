//
// Created by xiongyi on 3/27/16.
//
#include <iostream>
#include "ros/ros.h"
#include <dirent.h>
#include "yaml-cpp/yaml.h"
#include <src/DataStructure/Frame.h>
#include <src/utility/ImageHelper.h>
#include "opencv2/opencv.hpp"

#include "sophus/sophus.hpp"
#include "Eigen/Eigen"
#include "utility/ParameterServer.cpp"
#include "tracker/Tracker.h"
#include "tracker/TrackRefFrame.h"
#include "utility/easylogging++.h"
INITIALIZE_EASYLOGGINGPP

//#include "cxy_lsdslam/cxy_lsdslam_param.h"
int getdir (std::string dir, std::vector<std::string> &files);

int main(int argc, char** argv)
{


    ros::init(argc, argv, "cxy_lsdslam");
    el::Configurations conf("/home/xiongyi/workspace/src/cxy-LSD-SLAM/cfg/LoggerConfig.conf");
    el::Loggers::reconfigureAllLoggers( conf );

    const float fx = cxy::ParameterServer::getParameter<float>("fx");
    const float fy = cxy::ParameterServer::getParameter<float>("fy");
    const float cx = cxy::ParameterServer::getParameter<float>("cx");
    const float cy = cxy::ParameterServer::getParameter<float>("cy");
    const float dist_1 = cxy::ParameterServer::getParameter<float>("dist_1");
    const float dist_2 = cxy::ParameterServer::getParameter<float>("dist_2");
    const float dist_3 = cxy::ParameterServer::getParameter<float>("dist_3");
    const float dist_4 = cxy::ParameterServer::getParameter<float>("dist_4");


//    Sophus::Matrix3f K;
//    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    std::string source_dir = cxy::ParameterServer::getParameter<std::string>("file_dir");
    std::string rgb_dir = "rgb";
    std::string depth_dir = "depth";
    std::vector<std::string> rgb_files;
    std::vector<std::string> depth_files;
    ROS_INFO_STREAM("RGB mImage dir : "<<(source_dir+rgb_dir));
    if(getdir(source_dir+rgb_dir, rgb_files) <= 0)
    {
        printf("found %d rgb_files, the first one is %s\n", (int)rgb_files.size(), rgb_files[0].c_str());
    }
    if(getdir(source_dir+depth_dir, depth_files) <= 0)
    {
        printf("found %d depth_files, the first one is %s\n", (int)depth_files.size(), depth_files[0].c_str());
    }
    assert(rgb_files.size() == depth_files.size());

    //// initializ
    cv::Mat map1, map2;
    cv::Mat imgTmp = cv::imread(rgb_files[0], CV_LOAD_IMAGE_GRAYSCALE);
    int width = imgTmp.cols;
    int height = imgTmp.rows;
    cv::Mat K = cv::Mat(3, 3, CV_32F, cv::Scalar(0));
    cv::Mat distCoeffs = cv::Mat(4, 1, CV_32F, cv::Scalar(0));
    cv::Mat K_new = cv::Mat(3, 3, CV_32F, cv::Scalar(0));
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.at<float>(2, 2) = 1;

    distCoeffs.at<float>(0) = dist_1;
    distCoeffs.at<float>(1) = dist_2;
    distCoeffs.at<float>(2) = dist_3;
    distCoeffs.at<float>(3) = dist_4;
//    K_1.at<double>(0, 0) = fx * mWidth;
//    K_1.at<double>(1, 1) = fy * height;
//    K_1.at<double>(0, 2) = cx * mWidth;
//    K_1.at<double>(1, 2) = cy * height;
    assert(imgTmp.type() == CV_8U);



    K_new = cv::getOptimalNewCameraMatrix(K, distCoeffs, imgTmp.size(), 0);
    Eigen::Matrix3f K_new_eigen;
    K_new_eigen.setZero();
    K_new_eigen(0, 0) = K_new.at<float>(0, 0);
    K_new_eigen(0, 2) = K_new.at<float>(0, 2);
    K_new_eigen(1, 1) = K_new.at<float>(1, 1);
    K_new_eigen(1, 2) = K_new.at<float>(1, 2);
    K_new_eigen(2, 2) = K_new.at<float>(2, 2);

//    ROS_INFO("%f %f %f %f", K.at<float>(0,0), K.at<float>(1,1), K.at<float>(0,2), K.at<float>(1,2));
//    ROS_INFO("%f %f %f %f", K_new.at<float>(0,0), K_new.at<float>(1,1), K_new.at<float>(0,2), K_new.at<float>(1,2));
    cv::initUndistortRectifyMap(K, distCoeffs, cv::Mat(), K_new, imgTmp.size(), CV_32FC1, map1, map2);
//    ROS_INFO("%f %f %f %f", distCoeffs.at<float>(0), distCoeffs.at<float>(1), distCoeffs.at<float>(2), distCoeffs.at<float>(3));
//    ROS_INFO("%f %f %f %f", map1.at<float>(0,0), map1.at<float>(1,1), map1.at<float>(10,20), map1.at<float>(100,200));
    cv::waitKey(0);
    double timeStampFake = 0;
    const double timeInterval = 0.03;
    bool isInitialized = false;
    cxy::Tracker tracker(width, height);
    cxy::TrackRefFrame* trackRefFramePtr = nullptr;
    Sophus::SE3f frameToRef;

    cv::Mat imageTrackRef;
    for (int ii = 0; ii < rgb_files.size(); ++ii)
    {
        cv::Mat imgRaw = cv::imread(rgb_files[ii], CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat imgDepthRaw = cv::imread(depth_files[ii], -1);
//        cv::imshow( "depthRaw", imgDepthRaw);
        cv::Mat imgDepth;
        cxy::ImageHelper::convertRawDepthImage(imgDepthRaw, imgDepth);
        //ROS_INFO("Depth mImage at: %f %f %f", imgDepth.at<float>(50, 100), imgDepth.at<float>(100,100), imgDepth.at<float>(300,300));

//        cxy::DebugUtility::DisplayImage(imgDepth, "depthImage");
        cv::Mat imgUndistort;
        cv::remap(imgRaw, imgUndistort, map1, map2, cv::INTER_LINEAR);


        cxy::Frame frame(ii, width, height, K_new_eigen, timeStampFake, imgUndistort.data );
        if ( ! isInitialized)
        {
            frame.setDepth(imgDepth.data, false);
            trackRefFramePtr = new cxy::TrackRefFrame(&frame);
            isInitialized = true;
            imageTrackRef = imgUndistort;
            continue;
        }
        cv::imshow("imgUndistort", imgUndistort);
        cv::imshow("ImageTrackReference", imageTrackRef);
        cv::waitKey(0);
        tracker.track_NoDepth(trackRefFramePtr, &frame, frameToRef);

        timeStampFake += timeInterval;

    }


    return 0;

}


int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);

        if(name != "." && name != "..")
            files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
    for(unsigned int i=0;i<files.size();i++)
    {
        if(files[i].at(0) != '/')
            files[i] = dir + files[i];
    }

    return files.size();
}
