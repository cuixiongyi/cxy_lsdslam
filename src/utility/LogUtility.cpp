//
// Created by xiongyi on 4/14/16.
//

#include "LogUtility.h"

void LogUtility::writeMatToLog(const cv::Mat& image, const std::string &name)
{
    LOG(DEBUG)<<"Log Mat image: "<<name<<"\nwidth: "<<image.cols<<" height: "<<image.rows<<
             " type: "<<image.type();
    std::stringstream ss;
    for (int ii = 0; ii < image.rows; ++ii) {
        for (int jj = 0; jj < image.cols; ++jj) {
            ss<<image.at<float>(ii,jj)<<" ";
        }
        ss<<std::endl;
    }
    LOG(DEBUG)<<ss.str();

}

void LogUtility::writeMatToLog(int width, int height, int type, void *data, const std::string& name) {
    cv::Mat image(cv::Size(width, height), type, data);
    writeMatToLog(image, name);
}



