//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_FRAME_H
#define CXY_LSDSLAM_FRAME_H


#include <vector>
#include <memory>
#include "Eigen/Eigen"

#include <src/utility/MemoryManager.h>
#include "DataStructure/DataTypeDeclearation.h"
#include "utility/ParameterServer.h"
#include "utility/DebugUtility.h"
#include "FramePoseStruct.h"
#include "utility/LogUtility.h"

namespace cxy{



    class Frame {


    private:
/// in seconds
        int mFrameid = 0;
        double mTimeStamp = 0.0;

        using Pointer_Vector_bool = cxy::ArrayPointer_Vector<bool>;

        //// mImage data
        std::vector<int> mWidth, mHeight;
        std::vector<Eigen::Matrix3f> mK, mKInv;
        std::vector<float> mFx, mFy, mCx, mCy;
        std::vector<float> mFxInv, mFyInv, mCxInv, mCyInv;
        ArrayPointer_Vector<float> mImage;
        Pointer_Vector_bool mImageValid;
        ArrayPointer_Vector<Eigen::Vector4f> mGradient;
        Pointer_Vector_bool mGradientValid;
        ArrayPointer_Vector<float> mMaxGradients;
        Pointer_Vector_bool maxGradientsValid;
        /// std::unique_ptr<int[]> my_array(new int[5]);
        // negative depthvalues are actually allowed, so setting this to -1 does NOT invalidate the pixel's depth.
        // a pixel is valid iff mIdepthVar[i] > 0. Reference LSD-SLAM_core/Frame.h
        ArrayPointer_Vector<float> mIdepth;
        Pointer_Vector_bool mIdepthValid;
        // MUST contain -1 for invalid pixel (that dont have depth)!!
        ArrayPointer_Vector<float> mIdepthVar;
        Pointer_Vector_bool mIdepthVarValid;

        const int _MaxImagePyramidLevel;

        void buildImagePyramid(int level);
        void buildImage(int level);
        void buildGradient(int level);
        void buildMaxGradient(int level);
        void buildInvDepthPyramid(int level);
        void buildIDepthMap_Var(int level);

        bool isHasDepth = false;

        void convertRawDepthImage(const cv::Mat& input, cv::Mat& output, float scale);

    public:
        float mInitialTrackedResidual = 0.f;
        std::unique_ptr<FramePoseStruct> pose;


    public:

        Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image);

        void initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image);
        void setDepth(uchar* idepth, bool isInversDepth, uchar* idepthVar = nullptr);

        const int& getWidth(int level) const { return mWidth[level];}
        const int& getHeight(int level) const { return mHeight[level];}
        const float& getFxInv(int level) const { return mFxInv[level];}
        const float& getFyInv(int level) const { return mFyInv[level];}
        const float& getCxInv(int level) const { return mCxInv[level];}
        const float& getCyInv(int level) const { return mCyInv[level];}

        const float& getFx(int level) const { return mFx[level];}
        const float& getFy(int level) const { return mFy[level];}
        const float& getCx(int level) const { return mCx[level];}
        const float& getCy(int level) const { return mCy[level];}

        inline float const* getImage(int level) const { return mImage[level].get();};
        inline Eigen::Vector4f const* getGradient(int level) const { return mGradient[level].get();};
        inline float const* getMaxGradients(int level) const { return mMaxGradients[level].get();};
        inline float const* getIdepth(int level) const { return mIdepth[level].get();};
        inline float const* getIdepthVar(int level) const { return mIdepthVar[level].get();};









    public:
        inline int getFrameid() const {return mFrameid;}

        inline double getTimeStamp() const {return mTimeStamp;}
    };

}


#endif //CXY_LSDSLAM_FRAME_H
