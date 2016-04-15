//
// Created by xiongyi on 3/31/16.
//

#include <highgui.h>
#include <opencv2/opencv.hpp>
#include "DebugUtility.h"

namespace cxy
{
    bool DebugUtility::mIsPublishThreadShouldStop = false;
    std::unique_ptr<ros::Publisher> DebugUtility::pubPointCloud = nullptr;
    std::unique_ptr<sensor_msgs::PointCloud2> DebugUtility::mPointcloudData = nullptr;
    DebugUtility::DebugUtilityHandle* DebugUtility::DebugUtilityHandle::instance = nullptr;


    void DebugUtility::DisplayImage(int width, int height, int type, void* data, std::string windowName, bool keep, bool normalize)
    {
        /*
             * Debug: display the mImage
             */
        if (normalize && CV_32F == type)
        {
            float maximum = -10.0;
            float minimum = 999;
            auto dataPtr = (float*)data;
            auto minmaxRet = std::minmax_element(dataPtr, dataPtr+width*height);
            minimum = *minmaxRet.first;
            maximum = *minmaxRet.second;
            ROS_INFO("maximum in depth: %f", maximum);
            for (int ii = 0; ii < width*height; ++ii)
            {
                dataPtr[ii] = (dataPtr[ii] - minimum) / (maximum - minimum) * 255;
            }

        }
        cv::Mat imageTest(cv::Size(width, height), type, data);
        cv::Mat imageTest2;
        imageTest.convertTo(imageTest2, CV_8UC1);
//        cv::convertScaleAbs(imageTest, imageTest2);
        cv::imshow(windowName, imageTest2);
        cv::waitKey(0);

        if ( ! keep)
            cv::destroyWindow(windowName);
        return ;
    }

    void DebugUtility::DisplayImage(cv::Mat image, std::string windowName, bool keep)
    {
        /*
             * Debug: display the mImage
             */
        cv::Mat imageTest2;
        image.convertTo(imageTest2, CV_8U);
        cv::imshow(windowName, imageTest2);
        cv::waitKey(0);

        if ( ! keep)
            cv::destroyWindow(windowName);
        return ;
    }

    void DebugUtility::PublishPointCloudThread()
    {
        ros::Rate rate(5);
        while (true)
        {
            pubPointCloud->publish(*mPointcloudData);
            if (mIsPublishThreadShouldStop)
            {
                mIsPublishThreadShouldStop = false;
                break;
            }
            rate.sleep();
        }
    }


    void DebugUtility::PublishPointCloud(float const*const xPtr, float const*const yPtr, float const*const zPtr, float const*const intensityPtr, unsigned int size, const std::string& pointcloudName)
    {
        auto pubTmp = DebugUtilityHandle::getHandel().advertise<sensor_msgs::PointCloud2>(pointcloudName, 1);
        pubPointCloud = std::unique_ptr<ros::Publisher>(new ros::Publisher);
        *pubPointCloud = pubTmp;
        //pointcloudData = new std::unique_ptr<sensor_msgs::PointCloud2>();
        PointCloudXYZI::Ptr pointcloudXYZI(new PointCloudXYZI());
        pointcloudXYZI->header.frame_id = "world";
        pointcloudXYZI->width = pointcloudXYZI->height = 0;

        for (unsigned int ii = 0; ii < size; ++ii)
        {
            auto pointpcl = pcl::PointXYZI();
            pointpcl.x = xPtr[ii];
            pointpcl.y = yPtr[ii];
            pointpcl.z = zPtr[ii];
            pointpcl.intensity = intensityPtr[ii];
            pointcloudXYZI->push_back(pointpcl);

        }

        mPointcloudData = std::unique_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
        pcl::toROSMsg<pcl::PointXYZI>(*pointcloudXYZI, *mPointcloudData);
//        mPointcloudDataPtr = pointcloudXYZI
        mIsPublishThreadShouldStop = false;
        std::thread pubThread(&DebugUtility::PublishPointCloudThread);
        std::string tmp;
        std::cin>>tmp;

        mIsPublishThreadShouldStop = true;
    }
    void DebugUtility::PublishPointCloud(Eigen::Vector3f const *pointInput, unsigned int size, const std::string& pointcloudName)
    {
        auto pubTmp = DebugUtilityHandle::getHandel().advertise<sensor_msgs::PointCloud2>(pointcloudName, 1);
        pubPointCloud = std::unique_ptr<ros::Publisher>(new ros::Publisher);
        *pubPointCloud = pubTmp;
        //pointcloudData = new std::unique_ptr<sensor_msgs::PointCloud2>();
        PointCloudXYZI::Ptr pointcloudXYZI(new PointCloudXYZI());
        pointcloudXYZI->header.frame_id = "world";
        pointcloudXYZI->width = pointcloudXYZI->height = 0;


        for (unsigned int ii = 0; ii < size; ++ii)
        {
            const auto& point = pointInput[ii];
            auto pointpcl = pcl::PointXYZI();
            pointpcl.x = point(0);
            pointpcl.y = point(1);
            pointpcl.z = point(2);
            pointpcl.intensity = point(0);
            pointcloudXYZI->push_back(pointpcl);

        }

        mPointcloudData = std::unique_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
        pcl::toROSMsg<pcl::PointXYZI>(*pointcloudXYZI, *mPointcloudData);
//        mPointcloudDataPtr = pointcloudXYZI
        mIsPublishThreadShouldStop = false;
        std::thread pubThread(&DebugUtility::PublishPointCloudThread);
        std::string tmp;
        std::cin>>tmp;

        mIsPublishThreadShouldStop = true;
    }


}

