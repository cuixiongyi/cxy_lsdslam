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

        buildImagePyramid(3);

    }
    void Frame::buildImagePyramid(int ii)
    {
        for (int ii = 0; ii < ii; ++ii)
        {
            buildImage(ii);
            buildGraident(ii);
            buildIDepthMap(ii);
            buildIdepthVar(ii);
        }

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

    void Frame::buildImage(int ii)
    {
        if (0 == ii)
            return;
        mData.width.push_back(mData.width[ii-1]/2);
        mData.height.push_back(mData.height[ii-1]/2);
        mData.fx.push_back(mData.fx[ii-1] * 0.5);
        mData.fy.push_back(mData.fy[ii-1] * 0.5);
        mData.cx.push_back((mData.cx[0] + 0.5) / ((int)1<<ii) - 0.5);
        mData.cy.push_back((mData.cy[0] + 0.5) / ((int)1<<ii) - 0.5);

        Eigen::Matrix3f tmp;
        tmp.setZero();
        tmp  << mData.fx[ii], 0.0, mData.cx[ii], 0.0, mData.fy[ii], mData.cy[ii], 0.0, 0.0, 1.0;	// synthetic
        mData.K.push_back(tmp);
        mData.KInv.push_back(tmp.inverse());

        mData.fxInv.push_back(mData.KInv[ii](0,0));
        mData.fyInv.push_back(mData.KInv[ii](1,1));
        mData.cxInv.push_back(mData.KInv[ii](0,2));
        mData.cyInv.push_back(mData.KInv[ii](1,2));

        int width = mData.width[ii-1];
        int height = mData.height[ii-1];

        const float* source = mData.image[ii-1].get();
        float* dest = mData.image[ii-1].get();
        int wh = width*height;
        const float* s;

        for(int y=0;y<wh;y+=width*2)
        {
            for(int x=0;x<width;x+=2)
            {
                s = source + x + y;
                *dest = (s[0] +
                         s[1] +
                         s[width] +
                         s[1+width]) * 0.25f;
                dest++;
            }
        }

    }

    void Frame::buildGraident(int ii)
    {


    }


}