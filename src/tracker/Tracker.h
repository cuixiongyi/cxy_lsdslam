//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_TRACKER_H
#define CXY_LSDSLAM_TRACKER_H

#include "TrackRefFrame.h"
#include "sophus/se3.hpp"
#include "Optimization/NormalEquationLeastSquare.h"
#include "DataStructure/DataTypeDeclearation.h"

namespace cxy
{
    class Tracker {

    public:
        Tracker(const int& width, const int& height);

        SE3 track_NoDepth(TrackRefFrame const*const refTrackFrameInput,
                            Frame *const newFrameInput,
                            const Sophus::SE3f& initPose);
        int track_WithDepth();



        float getResidual_Buffer(const int& level,
                                   const cxy::TrackRefFrame *const refFrameInput,
                                   const cxy::Frame *const newFrameInput,
                                   const Eigen::Vector3f *const refPoint3DPtrInput,
                                   const Eigen::Vector2f *const refColorVarPtrInput,
                                   Sophus::SE3f &poseInput
        );

        float getWeight_Residual(const Sophus::SE3f &refToFrame);

        Vector6f getJacobian_Update(NormalEquationLeastSquare& ls);

    private:
        TrackRefFrame const* mRefTrackFrame = nullptr;
        Frame * mNewTrackFrame = nullptr;

        float pointUsage;
        float lastGoodCount;
        float lastMeanRes;
        float lastBadCount;
        float lastResidual;

        float affineEstimation_a = 1;
        float affineEstimation_b = 0;
        float affineEstimation_a_lastIt = 0;
        float affineEstimation_b_lastIt = 0;


        bool diverged;
        bool trackingWasGood;

        /// constant parameter
        float Const_Max_Pixel_Diff_Accept_Throush;
        float Const_Max_Diff_Grad_Multi;
        bool Const_UseAffineLightningEstimation = false;
        float Const_ImageColorVariance;
        float Const_IDepthVarianceWeight;
        float Const_Huber_D;
        std::vector<float> Const_LambdaInitialLevel;
        std::vector<int> Const_IterationNumLevel;
        std::vector<float> Const_ConvergenceEps;
        float Const_LambdaSuccessFac;
        std::vector<float> Const_StepSizeMin;
        float Const_LambdaFailFactor;




    private:



    private:

    private:
        ArrayPointer<float>   mBuf_warped_residual;
        ArrayPointer<float>   mBuf_warped_dx;
        ArrayPointer<float>   mBuf_warped_dy;
        ArrayPointer<float>   mBuf_warped_x;
        ArrayPointer<float>   mBuf_warped_y;
        ArrayPointer<float>   mBuf_warped_z;

        ArrayPointer<float>   mBuf_d;
        ArrayPointer<float>   mBuf_idepthVar;
        ArrayPointer<float>   mBuf_weight_p;
        ArrayPointer<bool>    mBuf_isPixelGood;

        unsigned int buf_warped_size;

    public:
        inline const float* getMBuf_warped_residual() const {return mBuf_warped_residual.get();}

        inline const float* getMBuf_warped_dx() const {return mBuf_warped_dx.get();}

        inline const float* getMBuf_warped_dy() const {return mBuf_warped_dy.get();}

        inline const float* getMBuf_warped_x() const {return mBuf_warped_x.get();}

        inline const float* getMBuf_warped_y() const {return mBuf_warped_y.get();}

        inline const float* getMBuf_warped_z() const {return mBuf_warped_z.get();}

        inline const float* getMBuf_d() const {return mBuf_d.get();}

        inline const float* getMBuf_idepthVar() const {return mBuf_idepthVar.get();}

        inline const float* getMBuf_weight_p() const {return mBuf_weight_p.get();}

        inline const bool* getMBuf_isPixelGood() const {return mBuf_isPixelGood.get();}
        inline int getBuf_warped_size() const {return buf_warped_size;}


        inline  float* getMBuf_warped_residual()  {return mBuf_warped_residual.get();}

        inline  float* getMBuf_warped_dx()  {return mBuf_warped_dx.get();}

        inline  float* getMBuf_warped_dy()  {return mBuf_warped_dy.get();}

        inline  float* getMBuf_warped_x()  {return mBuf_warped_x.get();}

        inline  float* getMBuf_warped_y()  {return mBuf_warped_y.get();}

        inline  float* getMBuf_warped_z()  {return mBuf_warped_z.get();}

        inline  float* getMBuf_d()  {return mBuf_d.get();}

        inline  float* getMBuf_idepthVar()  {return mBuf_idepthVar.get();}

        inline  float* getMBuf_weight_p()  {return mBuf_weight_p.get();}

        inline bool* getMBuf_isPixelGood()  {return mBuf_isPixelGood.get();}

    };


}


#endif //CXY_LSDSLAM_TRACKER_H
