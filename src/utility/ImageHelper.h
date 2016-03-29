//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_IMAGEHELPER_H
#define CXY_LSDSLAM_IMAGEHELPER_H


#include <opencv2/core/core.hpp>

class ImageHelper {
private:
    cv::Mat mImg;
public:

    void setImage(const cv::Mat& img);

    void getGradient();
};


#endif //CXY_LSDSLAM_IMAGEHELPER_H
