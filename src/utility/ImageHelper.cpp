//
// Created by xiongyi on 3/28/16.
//

#include "ImageHelper.h"



/**
* Converts the given raw depth mImage (type CV_16UC1) to a CV_32FC1 mImage rescaling every pixel with the given scale
* and replacing 0 with NaNs.
*/
void ImageHelper::convertRawDepthImage(const cv::Mat& input, cv::Mat& output, float scale)
{
    output.create(input.rows, input.cols, CV_32FC1);
    const unsigned short* input_ptr = (unsigned short*)input.data;
    float* output_ptr = (float*)output.data;

    for(int idx = 0; idx < input.size().area(); idx++, input_ptr++, output_ptr++)
    {
        auto& inputeVal = *input_ptr;
        if(inputeVal == 0)
        {
            *output_ptr = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            *output_ptr = (float)inputeVal * scale;
        }
    }
}