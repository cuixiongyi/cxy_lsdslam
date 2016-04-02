//
// Created by xiongyi on 3/31/16.
//

#include <highgui.h>
#include <opencv2/opencv.hpp>
#include "DebugUtility.h"

namespace cxy
{
    void DebugUtility::DisplayImage(int width, int height, int type, void* data, std::string windowName)
    {
        /*
             * Debug: display the image
             */
        cv::Mat imageTest(cv::Size(width, height), type, data);
        cv::Mat imageTest2;
        imageTest.convertTo(imageTest2, CV_8U);
        cv::imshow(windowName, imageTest2);
        cv::waitKey(0);
        cv::destroyWindow(windowName);
        return ;
    }

}

