//
// Created by xiongyi on 3/28/16.
//

#include "ImageHelper.h"

namespace cxy {

/**
* Converts the given raw depth mImage (type CV_16UC1) to a CV_32FC1 mImage rescaling every pixel with the given scale
* and replacing 0 with NaNs.
*/
    void ImageHelper::convertRawDepthImage(const cv::Mat &input, cv::Mat &output, float scale) {
        output.create(input.rows, input.cols, CV_32FC1);
        const unsigned short *input_ptr = (unsigned short *) input.data;
        float *output_ptr = (float *) output.data;

        for (int idx = 0; idx < input.size().area(); idx++, input_ptr++, output_ptr++) {
            auto &inputeVal = *input_ptr;
            if (inputeVal == 0) {
                *output_ptr = std::numeric_limits<float>::quiet_NaN();
            }
            else {
                *output_ptr = (float) inputeVal * scale;
            }
        }
    }


/// interpolate the grad and color
/// But I haven't checkout the math
    Eigen::Vector3f ImageHelper::getInterpolatedElement43(const Eigen::Vector4f *const gradPtrInput,
                                                                 const float x, const float y, const int width) {
        /* Only the first 3 element is used, probabily for effeciency
        * [0] is dx
        * [1] is dy
        * [2] is I, the image gray value
        */
        int ix = (int) x;
        int iy = (int) y;
        float dx = x - ix;
        float dy = y - iy;
        float dxdy = dx * dy;
        const Eigen::Vector4f *gradPtr = gradPtrInput + ix + iy * width;


        return dxdy * *(const Eigen::Vector3f *) (gradPtr + 1 + width)
               + (dy - dxdy) * *(const Eigen::Vector3f *) (gradPtr + width)
               + (dx - dxdy) * *(const Eigen::Vector3f *) (gradPtr + 1)
               + (1 - dx - dy + dxdy) * *(const Eigen::Vector3f *) (gradPtr);
    }


}
