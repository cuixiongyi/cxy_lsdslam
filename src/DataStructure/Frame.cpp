//
// Created by xiongyi on 3/28/16.
//

#include <opencv2/core/mat.hpp>
#include <highgui.h>
#include "Frame.h"

namespace cxy
{
    template <class T>
        Pointer<T> Frame::unique_ptr_allocator(unsigned int size)
    {
        return Pointer<T>(new T[size]);
    }

    Frame::Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image)
    {
        initialize(id, width, height, K, timestamp);
        mData.image.push_back(std::move(unique_ptr_allocator<float>(width*height)));
//        mData.image[0] = unique_ptr_allocator<float>(width*height);
//        mData.image[0] = std::unique_ptr<float, decltype(std::free)*>(reinterpret_cast<float*>(std::malloc(width*height*sizeof(float))), std::free);

        float* maxPt = mData.image[0].get() + mData.width[0]*mData.height[0];

        for(float* pt = mData.image[0].get(); pt < maxPt; pt++)
        {
            float tmp = *image;
            *pt = tmp;
            image++;
        }

        /*
         * Debug: display the image
         */
        cv::Mat imageTest(cv::Size(width, height), CV_32FC1, mData.image[0].get());
        cv::Mat imageTest2;
        imageTest.convertTo(imageTest2, CV_8U);
        cv::imshow("imagePointerTest", imageTest2);
        cv::waitKey(0);


    }

    void Frame::initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp)
    {
        this->mFrameid = id;
        this->mData.width.push_back(width);
        this->mData.height.push_back(height);
        this->mData.K.push_back(K);
        this->mData.fx.push_back(K(0,0));
        this->mData.fy.push_back(K(1,1));
        this->mData.cx.push_back(K(0,2));
        this->mData.cy.push_back(K(1,2));

        this->mData.KInv.push_back(K.inverse());
        this->mData.fxInv.push_back(this->mData.KInv[0](0,0));
        this->mData.fyInv.push_back(this->mData.KInv[0](1,1));
        this->mData.cxInv.push_back(this->mData.KInv[0](0,2));
        this->mData.cyInv.push_back(this->mData.KInv[0](1,2));

        this->mTimeStamp = timestamp;

    }

}