//
// Created by xiongyi on 3/27/16.
//
#include <iostream>
#include "ros/ros.h"
#include <dirent.h>
#include "opencv2/opencv.hpp"

#include "sophus/sophus.hpp"
#include "Eigen/Eigen"
#include "utility/ParameterServer.cpp"
#include "yaml-cpp/yaml.h"
//#include "cxy_lsdslam/cxy_lsdslam_param.h"
int getdir (std::string dir, std::vector<std::string> &files);

int main()
{


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
    std::string source;
    std::vector<std::string> files;
    source = "/home/xiongyi/workspace/src/lsd_slam/images";

    if(getdir(source, files) <= 0)
    {
        printf("found %d files, the first one is %s\n", (int)files.size(), files[0].c_str());
    }

    //// initializ
    cv::Mat map1, map2;
    cv::Mat imgTmp = cv::imread(files[0], CV_LOAD_IMAGE_GRAYSCALE);
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
//    K_1.at<double>(0, 0) = fx * width;
//    K_1.at<double>(1, 1) = fy * height;
//    K_1.at<double>(0, 2) = cx * width;
//    K_1.at<double>(1, 2) = cy * height;
    assert(imgTmp.type() == CV_8U);



    K_new = cv::getOptimalNewCameraMatrix(K, distCoeffs, imgTmp.size(), 0);
//    ROS_INFO("%f %f %f %f", K.at<float>(0,0), K.at<float>(1,1), K.at<float>(0,2), K.at<float>(1,2));
//    ROS_INFO("%f %f %f %f", K_new.at<float>(0,0), K_new.at<float>(1,1), K_new.at<float>(0,2), K_new.at<float>(1,2));
    cv::initUndistortRectifyMap(K, distCoeffs, cv::Mat(), K_new, imgTmp.size(), CV_32FC1, map1, map2);
//    ROS_INFO("%f %f %f %f", distCoeffs.at<float>(0), distCoeffs.at<float>(1), distCoeffs.at<float>(2), distCoeffs.at<float>(3));
//    ROS_INFO("%f %f %f %f", map1.at<float>(0,0), map1.at<float>(1,1), map1.at<float>(10,20), map1.at<float>(100,200));
    cv::waitKey(0);
    for(auto filename : files)
    {
        cv::Mat imgRaw = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat imgUndistort;
        cv::remap(imgRaw, imgUndistort, map1, map2, cv::INTER_LINEAR);
//        cv::imshow("imgRaw", imgRaw);
        cv::imshow("imgUndistort", imgUndistort);


        cv::waitKey(0);


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
