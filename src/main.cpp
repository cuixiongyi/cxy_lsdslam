//
// Created by xiongyi on 3/27/16.
//
#include <iostream>
#include "ros/ros.h"
#include <dirent.h>
#include "opencv2/opencv.hpp"

#include "sophus/sophus.hpp"
#include "Eigen/Eigen"
#include "yaml-cpp/yaml.h"
//#include "cxy_lsdslam/cxy_lsdslam_param.h"
int getdir (std::string dir, std::vector<std::string> &files);

int main()
{
    std::string yamlConfig = "/home/xiongyi/workspace/src/cxy-LSD-SLAM/cfg/config.yaml";
    YAML::Node config = YAML::LoadFile(yamlConfig);

    const float fx = config["fx"].as<double>();
    const float fy = config["fy"].as<double>();
    const float cx = config["cx"].as<double>();
    const float cy = config["cy"].as<double>();

    Sophus::Matrix3f K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    std::string source;
    std::vector<std::string> files;
    source = "/home/xiongyi/workspace/src/lsd_slam/images";

    if(getdir(source, files) <= 0)
    {
        printf("found %d files, the first one is %s\n", (int)files.size(), files[0].c_str());
    }

    for(auto filename : files)
    {
        cv::Mat imgRaw = cv::imread(filename);
        cv::imshow("imgRaw", imgRaw);
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
