//
// Created by xiongyi on 3/31/16.
//

#ifndef CXY_LSDSLAM_DEBUGUTILITY_H
#define CXY_LSDSLAM_DEBUGUTILITY_H


#include <opencv2/core/hal/interface.h>
#include "opencv2/opencv.hpp"

namespace cxy
{
    class DebugUtility {

    public:
        static void DisplayImage(int width, int height, int type,  void* data, std::string windowName, bool keep = false);
        static void DisplayImage(cv::Mat image, std::string windowName, bool keep = false);


    };

}


#endif //CXY_LSDSLAM_DEBUGUTILITY_H
