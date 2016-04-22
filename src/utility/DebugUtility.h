//
// Created by xiongyi on 3/31/16.
//

#ifndef CXY_LSDSLAM_DEBUGUTILITY_H
#define CXY_LSDSLAM_DEBUGUTILITY_H


//#include <opencv2/core/hal/interface.h>
#include <thread>
#include <algorithm>
#include <functional>
#include "opencv2/opencv.hpp"
#include <Eigen/Eigen>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/builtin_float.h"
#include "DataStructure/DataTypeDeclearation.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "utility/LogUtility.h"
//#include <pcl_conversions>
#include "utility/MemoryManager.h"
namespace cxy
{
    class DebugUtility {

    public:
        static void DisplayImage(int width, int height, int type,  void* data, std::string windowName, bool keep = false, bool normalize = false, bool log = false);
        static void DisplayImage(cv::Mat image, std::string windowName, bool keep = false);

        static void PublishPointCloud(Eigen::Vector3f const *pointInput, unsigned int size, const std::string& pointcloudName);
        static void PublishPointCloud(float const*const xPtr, float const*const yPtr, float const*const zPtr, float const*const intensityPtr, unsigned int size, const std::string& pointcloudName);
        static void PublishPointCloud(const std::vector<PointXYZIf>& pointInput, const std::string& pointcloudName, const float& time);


        //        static sensor_msgs::PointCloud
        static void PublishPointCloudThread(const float& time=1.0);
        static bool mIsPublishThreadShouldStop;


        static std::unique_ptr<ros::Publisher> pubPointCloud;
//        static PointCloudXYZI::Ptr mPointcloudDataPtr;
        static std::unique_ptr<sensor_msgs::PointCloud2> mPointcloudData;

        class DebugUtilityHandle
        {
            static DebugUtilityHandle* instance;
            ros::NodeHandle nh;

            DebugUtilityHandle()
            {
                nh = ros::NodeHandle();
            }
        public:
            static DebugUtilityHandle* getInstance()
            {
                if (nullptr == instance)
                {
                    instance = new DebugUtilityHandle();
                }

                return instance;
            }
        public:
            static ros::NodeHandle& getHandel()
            {
                return getInstance()->nh;
            }
        };
    };
//    cxy::DebugUtility::DebugUtilityHandle* cxy::DebugUtility::DebugUtilityHandle::instance = nullptr;


}


#endif //CXY_LSDSLAM_DEBUGUTILITY_H
