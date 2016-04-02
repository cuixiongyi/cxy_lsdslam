//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_FRAME_H
#define CXY_LSDSLAM_FRAME_H


#include <vector>
#include <memory>
#include "Eigen/Eigen"

#include "utility/ParameterServer.h"
#include "utility/DebugUtility.h"

namespace cxy{

    template <class T>
    using ArrayPointer = std::unique_ptr<T[]>;

    /// vector to a unique_ptr
    /// unique_ptr point to a trunk of memory: std::unique_ptr<int[]> my_array(new int[5]);
    template <class T>
    using ArrayPointer_Vector = std::vector<ArrayPointer<T> >;


    class Frame {

    private:

        /// in seconds
        int mFrameid = 0;
        double mTimeStamp = 0.0;

        struct Data
        {
            using Pointer_Vector_bool = cxy::ArrayPointer_Vector<bool>;

            std::vector<int> width, height;

            std::vector<Eigen::Matrix3f> K, KInv;
            std::vector<float> fx, fy, cx, cy;
            std::vector<float> fxInv, fyInv, cxInv, cyInv;

            ArrayPointer_Vector<float> image;
            Pointer_Vector_bool imageValid;

            ArrayPointer_Vector<Eigen::Vector4f> gradient;
            Pointer_Vector_bool gradientValid;

            ArrayPointer_Vector<float> maxGradients;
            Pointer_Vector_bool maxGradientsValid;

            /// std::unique_ptr<int[]> my_array(new int[5]);
            // negative depthvalues are actually allowed, so setting this to -1 does NOT invalidate the pixel's depth.
            // a pixel is valid iff idepthVar[i] > 0. Reference LSD-SLAM_core/Frame.h
            ArrayPointer_Vector<float> idepth;
            Pointer_Vector_bool idepthValid;

            // MUST contain -1 for invalid pixel (that dont have depth)!!
            ArrayPointer_Vector<float> idepthVar;
            Pointer_Vector_bool idepthVarValid;

        };
        Data mData;

        template <typename T>
        ArrayPointer<T> ArrayPointer_Allocator(unsigned int size);

        void buildImagePyramid(int level);
        void buildImage(int level);
        void buildGradient(int level);
        void buildMaxGradient(int level);
        void buildInvDepthPyramid(int level);
        void buildIDepthMap_Var(int level);

        bool isHasDepth = false;
        void setDepth(uchar* idepth, bool isInversDepth, uchar* idepthVar = nullptr);


    public:

        Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image);

        void initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image);


    };

}


#endif //CXY_LSDSLAM_FRAME_H
