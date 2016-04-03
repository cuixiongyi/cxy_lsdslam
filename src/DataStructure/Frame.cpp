//
// Created by xiongyi on 3/28/16.
//

#include <opencv2/core/mat.hpp>
#include <highgui.h>
#include <iostream>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "Frame.h"

namespace cxy
{
    template <class T>
        ArrayPointer<T> Frame::ArrayPointer_Allocator(unsigned int size)
    {
        return ArrayPointer<T>(new T[size]);
    }

    Frame::Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image)
    : _MaxImagePyramidLevel(3)
    {
        initialize(id, width, height, K, timestamp, image);
//        mData.image[0] = ArrayPointer_Allocator<float>(width*height);
//        mData.image[0] = std::unique_ptr<float, decltype(std::free)*>(reinterpret_cast<float*>(std::malloc(width*height*sizeof(float))), std::free);


        buildImagePyramid(_MaxImagePyramidLevel);

    }
    void Frame::buildImagePyramid(int level)
    {
        for (int ii = 0; ii < level; ++ii)
        {
            buildImage(ii);
            //cxy::DebugUtility::DisplayImage(mData.width[ii], mData.height[ii], CV_32FC1, mData.image[ii].get(), "PyramidTest");
            buildGradient(ii);
            buildMaxGradient(ii);
            //cxy::DebugUtility::DisplayImage(mData.width[ii], mData.height[ii], CV_32FC1, mData.maxGradients[ii].get(), "MaxGradient", true);

            /// test gradient image
            /*
            const float display_factor = ParameterServer::getParameter<float>("gradient_display_factor");
            unsigned int size = mData.width[ii]*mData.height[ii];
            auto gradient = ArrayPointer_Allocator<float>(size);
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
        for (int i = 0; i < level; ++i) {
            buildIDepthMap_Var(i);
        }
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


        mData.image.push_back(std::move(ArrayPointer_Allocator<float>(width * height)));
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

        mData.image.push_back(std::move(ArrayPointer_Allocator<float>(width * height)));
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

    void Frame::buildGradient(int level)
    {
        int width = mData.width[level];
        int height = mData.height[level];
        const float* img_pt = mData.image[level].get() + width;
        const float* img_pt_max = mData.image[level].get()  + width*(height-1);
        mData.gradient.push_back(std::move(ArrayPointer_Allocator<Eigen::Vector4f>(width * height)));
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

    void Frame::buildIDepthMap_Var(int level)
    {

        if ( ! isHasDepth)
            return;
        if (0 == level)
            return;
        auto width = mData.width[level];
        auto height = mData.height[level];
        auto size = width*height;

        mData.idepth.push_back(std::move(ArrayPointer_Allocator<float>(size)));
        mData.idepthVar.push_back(std::move(ArrayPointer_Allocator<float>(size)));

        float *const idepthPointerStart = mData.idepth[level].get();
        float *const idepthVarPointerStart = mData.idepthVar[level].get();

        int sw = mData.width[level - 1];

        const float* idepthSourceStart = mData.idepth[level - 1].get();
        const float* idepthSource = idepthSourceStart;
        const float* idepthVarSourceStart = mData.idepthVar[level - 1].get();
        const float* idepthVarSource = idepthVarSourceStart;
        float* idepthDest = idepthPointerStart;
        float* idepthVarDest = idepthVarPointerStart;

        for(int y=0;y<height;y++)
        {
            for(int x=0;x<width;x++)
            {
                int idx = 2*(x+y*sw);
                int idxDest = (x+y*width);

                float idepthSumsSum = 0;
                float ivarSumsSum = 0;
                int num=0;

                // build sums
                float ivar;
                float var = idepthVarSource[idx];
                if(var > 0)
                {
                    ivar = 1.0f / var;
                    ivarSumsSum += ivar;
                    idepthSumsSum += ivar * idepthSource[idx];
                    num++;
                }

                var = idepthVarSource[idx+1];
                if(var > 0)
                {
                    ivar = 1.0f / var;
                    ivarSumsSum += ivar;
                    idepthSumsSum += ivar * idepthSource[idx+1];
                    num++;
                }

                var = idepthVarSource[idx+sw];
                if(var > 0)
                {
                    ivar = 1.0f / var;
                    ivarSumsSum += ivar;
                    idepthSumsSum += ivar * idepthSource[idx+sw];
                    num++;
                }

                var = idepthVarSource[idx+sw+1];
                if(var > 0)
                {
                    ivar = 1.0f / var;
                    ivarSumsSum += ivar;
                    idepthSumsSum += ivar * idepthSource[idx+sw+1];
                    num++;
                }

                if(num > 0)
                {
                    float depth = ivarSumsSum / idepthSumsSum;
                    idepthDest[idxDest] = 1.0f / depth;
                    idepthVarDest[idxDest] = num / ivarSumsSum;
                    //ROS_INFO("Frame::setDepth: \nDepth vs IDepth");
                    //ROS_INFO("%f %f \n%f %f \n %f %f\n", idepthPointerStart[idxDest], idepthSourceStart[idx], idepthPointerStart[idxDest], idepthSourceStart[idx], idepthPointerStart[idxDest], idepthSourceStart[idx] );
                }
                else
                {
                    idepthDest[idxDest] = -1;
                    idepthVarDest[idxDest] = -1;
                }
            }
        }

        DebugUtility::DisplayImage(width, height, CV_32F, mData.idepth[level].get(), "idepthPyramid");
    }

    void Frame::setDepth(uchar* idepthInput, bool isInversDepth, uchar* idepthVarInput) {
        static const float _IDepthVar = ParameterServer::getParameter<float>("IDepthVar");
        std::function<float(float const&)> depthFunc = nullptr;
        std::function<float(float const&)> depthVarFunc = nullptr;
        if (isInversDepth)
        {
            depthFunc = [](float const& x)  {return x;};
        }
        else
        {
            depthFunc = [](float const& x)  {return 1 / x;};
        }
        if (nullptr != idepthVarInput)
        {
            depthVarFunc = [](float const& x)  {return x;};
        }
        else
        {
            depthVarFunc = [&](float const& x)  {return _IDepthVar;};
        }
        auto idepthInput2 = reinterpret_cast<float *>(idepthInput);
        auto idepthVarInput2 = reinterpret_cast<float *>(idepthVarInput);
//        idepthVarInput = (float*)idepthVarInput;
        auto width = mData.width[0];
        auto height = mData.height[0];
        auto size = width * height;
        mData.idepth.push_back(std::move(ArrayPointer_Allocator<float>(size)));
        mData.idepthVar.push_back(std::move(ArrayPointer_Allocator<float>(size)));

        float *const idepthPointerStart = mData.idepth[0].get();
        float *const idepthVarPointerStart = mData.idepthVar[0].get();
        float *const idepthPointerEnd = idepthPointerStart + size;

        float *idepthItr = idepthPointerStart;
        float *idepthVarItr = idepthVarPointerStart;
        for (; idepthItr < idepthPointerEnd;)
        {
            /// assuming the input depth is metric
            *idepthItr = depthFunc(*idepthInput2);
            /// if depth is nan set the var to -1
            *idepthVarItr = std::isnan(*idepthItr) ? -1 : depthVarFunc(*idepthVarInput2);
            ++idepthItr;
            ++idepthInput2;
            ++idepthVarItr;
            ++idepthVarInput2;
        }
        ROS_INFO("Frame::setDepth: \nDepth vs IDepth");

        int idx1 = 100*width + 100;
        int idx2 = 200*width + 200;
        int idx3 = 400*width + 300;
        idepthInput2 = reinterpret_cast<float *>(idepthInput);
        ROS_INFO("%f %f \n%f %f \n %f %f\n", idepthInput2[idx1], idepthPointerStart[idx1], idepthInput2[idx2], idepthPointerStart[idx2], idepthInput2[idx3], idepthPointerStart[idx3] );
        assert(depthFunc(idepthInput2[idx1]) == idepthPointerStart[idx1]);
        assert(depthFunc(idepthInput2[idx2]) == idepthPointerStart[idx2]);
        assert(depthFunc(idepthInput2[idx3]) == idepthPointerStart[idx3]);
        //DebugUtility::DisplayImage(width, height, CV_32F, mData.idepth[0].get(), "idepth");

        isHasDepth = true;
        for (int ii = 1; ii < _MaxImagePyramidLevel; ++ii)
        {
            buildIDepthMap_Var(ii);
        }
    }

    void Frame::buildMaxGradient(int level) {
        static float _Graident_Threshold = ParameterServer::getParameter<float>("Graident_Threshold");

        const int width = mData.width[level];
        const int height = mData.height[level];

        mData.maxGradients.push_back(std::move(ArrayPointer_Allocator<float>(width * height)));
        float *const _MaxGradientPointer = mData.maxGradients[level].get();

        auto gradientTmpPointer = ArrayPointer_Allocator<float>(width * height);
        float *const _maxGradTempPointer = gradientTmpPointer.get();


        // 1. write abs gradients in real mData.
        Eigen::Vector4f* gradxyii_pt = mData.gradient[level].get() + width;
        float* maxgrad_pt = _MaxGradientPointer + width;
        float* maxgrad_pt_max = maxgrad_pt + width*(height-1);

        for(; maxgrad_pt < maxgrad_pt_max; maxgrad_pt++, gradxyii_pt++)
        {
            float dx = *((float*)gradxyii_pt);
            float dy = *(1+(float*)gradxyii_pt);
            *maxgrad_pt = sqrtf(dx*dx + dy*dy);
        }

        // 2. smear up/down direction into temp buffer
        maxgrad_pt = _MaxGradientPointer + width + 1;
        maxgrad_pt_max = _MaxGradientPointer + width*(height-1)-1;
        float* maxgrad_t_pt = _maxGradTempPointer + width+1;
        for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
        {
            float g1 = maxgrad_pt[-width];
            float g2 = maxgrad_pt[0];
            if(g1 < g2) g1 = g2;
            float g3 = maxgrad_pt[width];
            if(g1 < g3)
                *maxgrad_t_pt = g3;
            else
                *maxgrad_t_pt = g1;
        }

        float numMappablePixels = 0;
        // 2. smear left/right direction into real mData
        maxgrad_pt = _MaxGradientPointer + width+1;
        maxgrad_pt_max = _MaxGradientPointer + width*(height-1)-1;
        maxgrad_t_pt = _maxGradTempPointer + width+1;
        for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
        {
            float g1 = maxgrad_t_pt[-1];
            float g2 = maxgrad_t_pt[0];
            if(g1 < g2) g1 = g2;
            float g3 = maxgrad_t_pt[1];
            if(g1 < g3)
            {
                *maxgrad_pt = g3;
                if(g3 >= _Graident_Threshold)
                    numMappablePixels++;
            }
            else
            {
                *maxgrad_pt = g1;
                if(g1 >= _Graident_Threshold)
                    numMappablePixels++;
            }
        }

    }



}