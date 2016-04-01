//
// Created by xiongyi on 3/28/16.
//

#include <opencv2/core/mat.hpp>
#include <highgui.h>
#include <iostream>
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
        initialize(id, width, height, K, timestamp, image);
//        mData.image[0] = unique_ptr_allocator<float>(width*height);
//        mData.image[0] = std::unique_ptr<float, decltype(std::free)*>(reinterpret_cast<float*>(std::malloc(width*height*sizeof(float))), std::free);


        buildImagePyramid(3);

    }
    void Frame::buildImagePyramid(int level)
    {
        for (int ii = 0; ii < level; ++ii)
        {
            buildImage(ii);
            //cxy::DebugUtility::DisplayImage(mData.width[ii], mData.height[ii], CV_32FC1, mData.image[ii].get(), "PyramidTest");
            buildGraident(ii);

            /// test gradient image
            /*
            const float display_factor = ParameterServer::getParameter<float>("gradient_display_factor");
            unsigned int size = mData.width[ii]*mData.height[ii];
            auto gradient = unique_ptr_allocator<float>(size);
            float* imagePointer = gradient.get();
            Eigen::Vector4f* gradPointer = mData.gradient[ii].get();
            for (int jj = 0; jj < size; ++jj) {
                *imagePointer = *((float*)gradPointer) * display_factor;
                //std::cout<<*imagePointer<<std::endl;
                imagePointer++;
                gradPointer++;
            }
            imagePointer = gradient.get();
            cxy::DebugUtility::DisplayImage(mData.width[ii], mData.height[ii], CV_32FC1, imagePointer, "Gradient_Test");
            */
        }

    }

    void Frame::buildInvDepthPyramid(int level)
    {
        buildIDepthMap(level);
        buildIdepthVar(level);
    }

    void Frame::initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image)
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


        mData.image.push_back(std::move(unique_ptr_allocator<float>(width*height)));
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
        cxy::DebugUtility::DisplayImage(width, height, CV_32FC1, mData.image[0].get(), "imagePointerTest");

    }

    void Frame::buildImage(int level)
    {
        if (0 == level)
            return;

        mData.width.push_back(mData.width[level-1]/2);
        mData.height.push_back(mData.height[level-1]/2);
        mData.fx.push_back(mData.fx[level-1] * 0.5);
        mData.fy.push_back(mData.fy[level-1] * 0.5);
        mData.cx.push_back((mData.cx[0] + 0.5) / ((int)1<<level) - 0.5);
        mData.cy.push_back((mData.cy[0] + 0.5) / ((int)1<<level) - 0.5);

        Eigen::Matrix3f tmp;
        tmp.setZero();
        tmp  << mData.fx[level], 0.0, mData.cx[level], 0.0, mData.fy[level], mData.cy[level], 0.0, 0.0, 1.0;	// synthetic
        mData.K.push_back(tmp);
        mData.KInv.push_back(tmp.inverse());

        mData.fxInv.push_back(mData.KInv[level](0,0));
        mData.fyInv.push_back(mData.KInv[level](1,1));
        mData.cxInv.push_back(mData.KInv[level](0,2));
        mData.cyInv.push_back(mData.KInv[level](1,2));

        int width = mData.width[level-1];
        int height = mData.height[level-1];

        mData.image.push_back(std::move(unique_ptr_allocator<float>(width*height)));
        const float* source = mData.image[level-1].get();
        float* dest = mData.image[level].get();
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

    void Frame::buildGraident(int level)
    {
        int width = mData.width[level];
        int height = mData.height[level];
        const float* img_pt = mData.image[level].get() + width;
        const float* img_pt_max = mData.image[level].get()  + width*(height-1);
        mData.gradient.push_back(std::move(unique_ptr_allocator<Eigen::Vector4f>(width*height)));
        Eigen::Vector4f* gradxyii_pt = mData.gradient[level].get() + width;

        // in each iteration i need -1,0,p1,mw,pw
        float val_m1 = *(img_pt-1); /// I(x-1,y)
        float val_00 = *img_pt;		/// I(x,y)
        float val_p1;				/// I(x+1, y)

        for(; img_pt < img_pt_max; img_pt++, gradxyii_pt++)
        {
            val_p1 = *(img_pt+1);

            *((float*)gradxyii_pt) = 0.5f*(val_p1 - val_m1);
            *(((float*)gradxyii_pt)+1) = 0.5f*(*(img_pt+width) - *(img_pt-width));
            *(((float*)gradxyii_pt)+2) = val_00;
            val_m1 = val_00;
            val_00 = val_p1;
        }

    }

    void Frame::buildIDepthMap(int level) {

    }

    void Frame::buildIdepthVar(int level) {

    }


}