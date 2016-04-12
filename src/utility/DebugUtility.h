//
// Created by xiongyi on 3/31/16.
//

#ifndef CXY_LSDSLAM_DEBUGUTILITY_H
#define CXY_LSDSLAM_DEBUGUTILITY_H


//#include <opencv2/core/hal/interface.h>
#include "opencv2/opencv.hpp"
#include <Eigen/Eigen>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
namespace cxy
{
    class DebugUtility {

    public:
        static void DisplayImage(int width, int height, int type,  void* data, std::string windowName, bool keep = false);
        static void DisplayImage(cv::Mat image, std::string windowName, bool keep = false);

        static void PublishPointClout(Eigen::Vector3f const* point);
//        static sensor_msgs::PointCloud

    private:
//        class DebugUtilityHandle
//        {
//            static DebugUtilityHandle* instance;
//            ros::NodeHandle nh;
//
//            DebugUtilityHandle()
//            {
//                nh = ros::NodeHandle();
//            }
//        public:
//            static DebugUtilityHandle* getInstance()
//            {
//                if (nullptr == instance)
//                {
//                    instance = new DebugUtilityHandle();
//                }
//
//                return instance;
//            }
//            ros::NodeHandle& getHandel()
//            {
//                return nh;
//            }
//        };
    };
//    cxy::DebugUtility::DebugUtilityHandle* cxy::DebugUtility::DebugUtilityHandle::instance = nullptr;


}


#endif //CXY_LSDSLAM_DEBUGUTILITY_H
