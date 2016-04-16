//
// Created by xiongyi on 4/14/16.
//

#ifndef CXY_LSDSLAM_LOGUTILITY_H
#define CXY_LSDSLAM_LOGUTILITY_H
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include "easylogging++.h"


namespace cxy
{
    class LogUtility {

    public:
        static void writeMatToLog(const cv::Mat& image, const std::string& name );
        static void writeMatToLog(int width, int height, int type, void* data, const std::string& name );

    };

}


#endif //CXY_LSDSLAM_LOGUTILITY_H
