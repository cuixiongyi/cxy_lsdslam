//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_FRAME_H
#define CXY_LSDSLAM_FRAME_H


#include <vector>
#include <memory>
#include "Eigen/Eigen"

#include "utility/ParameterServer.h"

namespace cxy{

    /// vector to a unique_ptr
    /// unique_ptr point to a trunk of memory: std::unique_ptr<int[]> my_array(new int[5]);
    template <typename T>
    using Pointer_Vector = std::vector<std::unique_ptr<T> >
    class Frame {

    private:

        /// in seconds
        int mFrameid = 0;
        double mTimeStamp = 0.0;

        struct Data
        {
            using Pointer_Vector_bool = cxy::Pointer_Vector<bool[]>;

            std::vector<int> width, height;

            std::vector<Eigen::Matrix3f> K, KInv;
            std::vector<float> fx, fy, cx, cy;
            std::vector<float> fxInv, fyInv, cxInv, cyInv;

            Pointer_Vector<float[]> image;
            Pointer_Vector_bool imageValid;

            Pointer_Vector<Eigen::Vector4f> gradient;
            Pointer_Vector_bool gradientValid;

            Pointer_Vector<float[]> maxGradients;
            Pointer_Vector_bool maxGradientsValid;

            /// std::unique_ptr<int[]> my_array(new int[5]);
            // negative depthvalues are actually allowed, so setting this to -1 does NOT invalidate the pixel's depth.
            // a pixel is valid iff idepthVar[i] > 0. Reference LSD-SLAM_core/Frame.h
            Pointer_Vector<float[]> idepth;
            Pointer_Vector_bool idepthValid;

            // MUST contain -1 for invalid pixel (that dont have depth)!!
            Pointer_Vector<float[]> idepthVar;
            Pointer_Vector_bool idepthVarValid;

        };
        Data mData;

    };

}


#endif //CXY_LSDSLAM_FRAME_H
