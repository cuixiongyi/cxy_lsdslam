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


    void DebugUtility::DisplayImage(int width, int height, int type, void* data, std::string windowName, bool keep, bool normalize, bool log)
    {
        /*
             * Debug: display the mImage
             */

        ArrayPointer<float> dataPtrUnique = MemoryManager::ArrayPointer_Allocator<float>(width*height);
        auto tmpPtr = (float*)data;
        auto dataPtr = dataPtrUnique.get();
        unsigned int size = width*height;
        for (int ii = 0; ii < size; ++ii)
        {
            dataPtr[ii] = tmpPtr[ii];
        }
        if (normalize && CV_32F == type)
        {
//            LogUtility::writeMatToLog(width, height, CV_32FC1, dataPtrUnique.get(), "depth before normalize");

            float maximum = *std::max_element(dataPtr, dataPtr+size, [](const float& a, const float& b) -> bool {
                if (std::isinf(a) || std::isnan(a))
                    return true;
                if (std::isinf(b) || std::isnan(b))
                    return false;
                return a < b;
            });
            float minimum = *std::min_element(dataPtr, dataPtr+size, [](const float& a, const float& b) -> bool {
                if (std::isinf(a) || std::isnan(a))
                    return false;
                if (std::isinf(b) || std::isnan(b))
                    return true;
                return a < b;
            });
            ROS_INFO("maximum in depth: %f", maximum);
            for (int ii = 0; ii < size; ++ii)
            {
                dataPtr[ii] = (dataPtr[ii] - minimum) / (maximum - minimum) * 255;
            }

//            LogUtility::writeMatToLog(width, height, CV_32FC1, dataPtr, "depth after normalize");


        }

        cv::Mat imageTest(cv::Size(width, height), type, dataPtr);
        cv::Mat imageTest2;
//        if (normalize)
//        {
//            cv::convertScaleAbs(imageTest, imageTest2);
//            imageTest2.copyTo(imageTest);
//        }

        imageTest.convertTo(imageTest2, CV_8UC1);
        if (log)
        {
            LogUtility::writeMatToLog(imageTest, windowName+"_log");
        }

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
        ros::Rate rate(1);
        while (true)
        {
//            LOG(INFO)<<"publish pointcloud";
            pubPointCloud->publish(*mPointcloudData);
            if (mIsPublishThreadShouldStop)
            {
//                LOG(INFO)<<"break publis pointcloud";

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
        pubThread.join();
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

        LOG(INFO)<<"signal to stop pointcloud";
        mIsPublishThreadShouldStop = true;
        pubThread.join();
        LOG(INFO)<<"publish pointcloud stopped";


    }

    void DebugUtility::PublishPointCloud(const std::vector<PointXYZIf>& pointInput, const std::string& pointcloudName)
    {
        auto pubTmp = DebugUtilityHandle::getHandel().advertise<sensor_msgs::PointCloud2>(pointcloudName, 1);
        pubPointCloud = std::unique_ptr<ros::Publisher>(new ros::Publisher);
        *pubPointCloud = pubTmp;
        //pointcloudData = new std::unique_ptr<sensor_msgs::PointCloud2>();
        PointCloudXYZI::Ptr pointcloudXYZI(new PointCloudXYZI());
        pointcloudXYZI->header.frame_id = "world";
        pointcloudXYZI->width = pointcloudXYZI->height = 0;
        pointcloudXYZI->reserve(pointInput.size());

        for (const auto& point : pointInput)
        {
            auto pointpcl = pcl::PointXYZI();
            pointpcl.x = point.x;
            pointpcl.y = point.y;
            pointpcl.z = point.z;
            pointpcl.intensity = point.intensity;
            pointcloudXYZI->push_back(pointpcl);

        }

        mPointcloudData = std::unique_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
        pcl::toROSMsg<pcl::PointXYZI>(*pointcloudXYZI, *mPointcloudData);
//        mPointcloudDataPtr = pointcloudXYZI
        mIsPublishThreadShouldStop = false;
        std::thread pubThread(&DebugUtility::PublishPointCloudThread);
        std::string tmp;
        std::cin>>tmp;

        LOG(INFO)<<"signal to stop pointcloud";
        mIsPublishThreadShouldStop = true;
        pubThread.join();
        LOG(INFO)<<"publish pointcloud stopped";

    }



}

