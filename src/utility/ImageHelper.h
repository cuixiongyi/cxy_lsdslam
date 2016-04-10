//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_IMAGEHELPER_H
#define CXY_LSDSLAM_IMAGEHELPER_H


#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>
#include <DataStructure/DataTypeDeclearation.h>

namespace cxy
{
    class ImageHelper {
    private:
        cv::Mat mImg;
    public:

        void setImage(const cv::Mat &img);

        void getGradient();

        static void convertRawDepthImage(const cv::Mat &input, cv::Mat &output, float scale = 1.035f / 5000.0f);

        static Eigen::Vector3f getInterpolatedElement43(const Eigen::Vector4f *const gradPtrInput, const float x,
                                                        const float y, const int width);

        static inline Sim3 sim3FromSE3(const SE3& se3, double scale)
        {
            Sim3 result(se3.unit_quaternion(), se3.translation());
            result.setScale(scale);
            return result;
        }


    };
}

#endif //CXY_LSDSLAM_IMAGEHELPER_H
