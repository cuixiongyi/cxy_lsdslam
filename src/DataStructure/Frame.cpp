//
// Created by xiongyi on 3/28/16.
//

#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <iostream>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "Frame.h"
#include "utility/MacroUtility.h"

namespace cxy
{


    Frame::Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image)
    : _MaxImagePyramidLevel(MAX_PYRAMID_LEVEL )
    {
        initialize(id, width, height, K, timestamp, image);
//        mImage[0] = ArrayPointer_Allocator<float>(mWidth*height);
//        mImage[0] = std::unique_ptr<float, decltype(std::free)*>(reinterpret_cast<float*>(std::malloc(mWidth*height*sizeof(float))), std::free);


        buildImagePyramid(_MaxImagePyramidLevel);
        pose = std::move(std::unique_ptr<FramePoseStruct>(new FramePoseStruct(this)));

    }
    void Frame::buildImagePyramid(int level)
    {
        for (int ii = 0; ii < level; ++ii)
        {
            buildImage(ii);
            cxy::DebugUtility::DisplayImage(mWidth[ii], mHeight[ii], CV_32FC1, mImage[ii].get(), "PyramidTest");
            buildGradient(ii);
            buildMaxGradient(ii);
            //cxy::DebugUtility::DisplayImage(mWidth[ii], height[ii], CV_32FC1, mMaxGradients[ii].get(), "MaxGradient", true);

            /// test mGradient mImage
            /*
            const float display_factor = ParameterServer::getParameter<float>("gradient_display_factor");
            unsigned int size = mWidth[ii]*height[ii];
            auto mGradient = ArrayPointer_Allocator<float>(size);
            float* imagePointer = mGradient.get();
            Eigen::Vector4f* gradPointer = mGradient[ii].get();
            for (int jj = 0; jj < size; ++jj) {
                *imagePointer = *((float*)gradPointer) * display_factor;
                //std::cout<<*imagePointer<<std::endl;
                imagePointer++;
                gradPointer++;
            }
            imagePointer = mGradient.get();
            cxy::DebugUtility::DisplayImage(mWidth[ii], height[ii], CV_32FC1, imagePointer, "Gradient_Test");
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
        this->mWidth.push_back(width);
        this->mHeight.push_back(height);
        this->mK.push_back(K);
        this->mFx.push_back(K(0,0));
        this->mFy.push_back(K(1,1));
        this->mCx.push_back(K(0,2));
        this->mCy.push_back(K(1,2));

        this->mKInv.push_back(K.inverse());
        this->mFxInv.push_back(this->mKInv[0](0,0));
        this->mFyInv.push_back(this->mKInv[0](1,1));
        this->mCxInv.push_back(this->mKInv[0](0,2));
        this->mCyInv.push_back(this->mKInv[0](1,2));

        this->mTimeStamp = timestamp;


        mImage.push_back(std::move(MemoryManager::ArrayPointer_Allocator<float>(width * height)));
        float* maxPt = mImage[0].get() + mWidth[0]*mHeight[0];
        for(float* pt = mImage[0].get(); pt < maxPt; pt++)
        {
            float tmp = *image;
            *pt = tmp;
            image++;
        }

        /*
         * Debug: display the mImage
         */
        //cxy::DebugUtility::DisplayImage(width, height, CV_32FC1, mImage[0].get(), "imagePointerTest");

    }

    void Frame::buildImage(int level)
    {
        if (0 == level)
            return;

        mWidth.push_back(mWidth[level-1]/2);
        mHeight.push_back(mHeight[level-1]/2);
        mFx.push_back(mFx[level-1] * 0.5);
        mFy.push_back(mFy[level-1] * 0.5);
        mCx.push_back((mCx[0] + 0.5) / ((int)1<<level) - 0.5);
        mCy.push_back((mCy[0] + 0.5) / ((int)1<<level) - 0.5);

        Eigen::Matrix3f tmp;
        tmp.setZero();
        tmp  << mFx[level], 0.0, mCx[level], 0.0, mFy[level], mCy[level], 0.0, 0.0, 1.0;	// synthetic
        mK.push_back(tmp);
        mKInv.push_back(tmp.inverse());

        mFxInv.push_back(mKInv[level](0,0));
        mFyInv.push_back(mKInv[level](1,1));
        mCxInv.push_back(mKInv[level](0,2));
        mCyInv.push_back(mKInv[level](1,2));

        int width = mWidth[level-1];
        int height = mHeight[level-1];

        mImage.push_back(std::move(MemoryManager::ArrayPointer_Allocator<float>(width * height)));
        const float* source = mImage[level-1].get();
        float* dest = mImage[level].get();
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
        int width = mWidth[level];
        int height = mHeight[level];
        const float* img_pt = mImage[level].get() + width;
        const float* img_pt_max = mImage[level].get()  + width*(height-1);
        mGradient.push_back(std::move(MemoryManager::ArrayPointer_Allocator<Eigen::Vector4f>(width * height)));
        Eigen::Vector4f* gradxyii_pt = mGradient[level].get() + width;

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
        auto width = mWidth[level];
        auto height = mHeight[level];
        auto size = width*height;

        mIdepth.push_back(std::move(MemoryManager::ArrayPointer_Allocator<float>(size)));
        mIdepthVar.push_back(std::move(MemoryManager::ArrayPointer_Allocator<float>(size)));

        int sourceWidth = mWidth[level - 1];

        const float* idepthSource = mIdepth[level - 1].get();
        const float* idepthVarSource = mIdepthVar[level - 1].get();
        float* idepthDest = mIdepth[level].get();
        float* idepthVarDest = mIdepthVar[level].get();

        for(int y=0;y<height;y++)
        {
            for(int x=0;x<width;x++)
            {
                int idx = 2*(x+y*sourceWidth);
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

                var = idepthVarSource[idx+sourceWidth];
                if(var > 0)
                {
                    ivar = 1.0f / var;
                    ivarSumsSum += ivar;
                    idepthSumsSum += ivar * idepthSource[idx+sourceWidth];
                    num++;
                }

                var = idepthVarSource[idx+sourceWidth+1];
                if(var > 0)
                {
                    ivar = 1.0f / var;
                    ivarSumsSum += ivar;
                    idepthSumsSum += ivar * idepthSource[idx+sourceWidth+1];
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

//        DebugUtility::DisplayImage(width, height, CV_32F, mIdepth[level].get(), "idepthPyramid", true, true);
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
        auto width = mWidth[0];
        auto height = mHeight[0];
        auto size = width * height;
        mIdepth.push_back(std::move(MemoryManager::ArrayPointer_Allocator<float>(size)));
        mIdepthVar.push_back(std::move(MemoryManager::ArrayPointer_Allocator<float>(size)));

        float *const idepthPointerStart = mIdepth[0].get();
        float *const idepthVarPointerStart = mIdepthVar[0].get();
        float *const idepthPointerEnd = idepthPointerStart + size;

        float *idepthItr = idepthPointerStart;
        float *idepthVarItr = idepthVarPointerStart;
        for (int ii = 0; idepthItr < idepthPointerEnd; ++ii)
        {
            /// assuming the input depth is metric
            *idepthItr = depthFunc(*idepthInput2);
            /// if depth is nan set the var to -1
            *idepthVarItr = std::isnan(*idepthItr) ? -1 : depthVarFunc(*idepthVarInput2);
//            if (std::isinf(*idepthItr))
//            {
//                *idepthItr = 1;
////                std::cout<<ii<<std::endl;
//            }
            ++idepthItr;
            ++idepthInput2;
            ++idepthVarItr;
            ++idepthVarInput2;
        }
        ROS_INFO("Frame::setDepth: \nDepth vs IDepth");

        int idx1 = 120*width + 105;
        int idx2 = 230*width + 205;
        int idx3 = 430*width + 305;
        idepthInput2 = reinterpret_cast<float *>(idepthInput);
        ROS_INFO("%f %f \n%f %f \n %f %f\n", idepthInput2[idx1], idepthPointerStart[idx1], idepthInput2[idx2], idepthPointerStart[idx2], idepthInput2[idx3], idepthPointerStart[idx3] );
        assert(depthFunc(idepthInput2[idx1]) == idepthPointerStart[idx1]);
        assert(depthFunc(idepthInput2[idx2]) == idepthPointerStart[idx2]);
        assert(depthFunc(idepthInput2[idx3]) == idepthPointerStart[idx3]);
//        DebugUtility::DisplayImage(width, height, CV_32FC1, idepthPointerStart, "mIdepth", true, true, true);
//        LogUtility::writeMatToLog(width, height, CV_32FC1, idepthPointerStart, "depth input");
        isHasDepth = true;
        for (int ii = 1; ii < _MaxImagePyramidLevel; ++ii)
        {
            buildIDepthMap_Var(ii);
        }
    }

    void Frame::buildMaxGradient(int level) {
        static float _Graident_Threshold = ParameterServer::getParameter<float>("Graident_Threshold");

        const int width = mWidth[level];
        const int height = mHeight[level];

        mMaxGradients.push_back(std::move(MemoryManager::ArrayPointer_Allocator<float>(width * height)));
        float *const _MaxGradientPointer = mMaxGradients[level].get();

        auto gradientTmpPointer = MemoryManager::ArrayPointer_Allocator<float>(width * height);
        float *const _maxGradTempPointer = gradientTmpPointer.get();


        // 1. write abs gradients in real 
        Eigen::Vector4f* gradxyii_pt = mGradient[level].get() + width;
        float* maxGradPtr = _MaxGradientPointer + width;
        float* maxgrad_pt_end = maxGradPtr + width*(height-1);

        for(; maxGradPtr < maxgrad_pt_end; maxGradPtr++, gradxyii_pt++)
        {
            float dx = *((float*)gradxyii_pt);
            float dy = *(1+(float*)gradxyii_pt);
            *maxGradPtr = sqrtf(dx*dx + dy*dy);
        }

        // 2. smear up/down direction into temp buffer
        maxGradPtr = _MaxGradientPointer + width + 1;
        maxgrad_pt_end = _MaxGradientPointer + width*(height-1)-1;
        float* maxGrad2Ptr = _maxGradTempPointer + width+1;
        for(;maxGradPtr<maxgrad_pt_end; maxGradPtr++, maxGrad2Ptr++)
        {
            float g1 = maxGradPtr[-width];
            float g2 = maxGradPtr[0];
            if(g1 < g2) g1 = g2;
            float g3 = maxGradPtr[width];
            if(g1 < g3)
                *maxGrad2Ptr = g3;
            else
                *maxGrad2Ptr = g1;
        }

        float numMappablePixels = 0;
        // 2. smear left/right direction into real mData
        maxGradPtr = _MaxGradientPointer + width+1;
        maxgrad_pt_end = _MaxGradientPointer + width*(height-1)-1;
        maxGrad2Ptr = _maxGradTempPointer + width+1;
        for(;maxGradPtr<maxgrad_pt_end; maxGradPtr++, maxGrad2Ptr++)
        {
            float g1 = maxGrad2Ptr[-1];
            float g2 = maxGrad2Ptr[0];
            if(g1 < g2) g1 = g2;
            float g3 = maxGrad2Ptr[1];
            if(g1 < g3)
            {
                *maxGradPtr = g3;
                if(g3 >= _Graident_Threshold)
                    numMappablePixels++;
            }
            else
            {
                *maxGradPtr = g1;
                if(g1 >= _Graident_Threshold)
                    numMappablePixels++;
            }
        }

    }



}