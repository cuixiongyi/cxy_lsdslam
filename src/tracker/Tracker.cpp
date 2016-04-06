//
// Created by xiongyi on 3/28/16.
//

#include <src/utility/ImageHelper.h>
#include "src/tracker/Tracker.h"

namespace cxy
{

}




cxy::Tracker::Tracker(const int& width, const int& height)
{

    // load Parameters
    Max_Pixel_Diff_Accept_Throush = ParameterServer::getParameter<float>("Max_Pixel_Diff_Accept_Throush");
    Max_Diff_Grad_Multi = ParameterServer::getParameter<float>("Max_Diff_Grad_Multi");
    const auto size = width*height;

    mBuf_warped_residual = MemoryManager::ArrayPointer_Allocator<float>(size);
    mBuf_warped_dx = MemoryManager::ArrayPointer_Allocator<float>(size);
    mBuf_warped_dy = MemoryManager::ArrayPointer_Allocator<float>(size);

    mBuf_warped_x = MemoryManager::ArrayPointer_Allocator<float>(size);
    mBuf_warped_y = MemoryManager::ArrayPointer_Allocator<float>(size);
    mBuf_warped_z = MemoryManager::ArrayPointer_Allocator<float>(size);

    mBuf_idepthVar = MemoryManager::ArrayPointer_Allocator<float>(size);
    mBuf_weight_p = MemoryManager::ArrayPointer_Allocator<float>(size);
    mBuf_d = MemoryManager::ArrayPointer_Allocator<float>(size);
    mBuf_isPixelGood = MemoryManager::ArrayPointer_Allocator<bool>(size);

}



int cxy::Tracker::track_NoDepth(const cxy::TrackRefFrame *const refFrameInput,
                                const cxy::Frame *const newFrameInput,
                                const Sophus::SE3f &frameToRefInput)
{

    mRefTrackFrame = refFrameInput;
    mNewTrackFrame = newFrameInput;

    Sophus::SE3f refToFramePose = frameToRefInput.inverse();

    //// loop over levels
 for (int ll = 0; ll < MAX_PYRAMID_LEVEL - 1; ++ll)
 {
     const auto width = newFrameInput->getWidth(ll);
     const auto height = newFrameInput->getHeight(ll);
     const auto size = width*height;
     //// init mBuf_isPixelGood
     for (int ii = 0; ii < size; ++ii)
         mBuf_isPixelGood[ii] = false;
     affineEstimation_a = 1; affineEstimation_b = 0;
     float lastResidual = 0.f;



     getResidual_Buffer(ll,
                        refFrameInput,
                        newFrameInput,
                        refFrameInput->getPoint3D(ll),
                        refFrameInput->getPointColor_Var(ll),
                        refToFramePose);
     if(useAffineLightningEstimation)
     {
         affineEstimation_a = affineEstimation_a_lastIt;
         affineEstimation_b = affineEstimation_b_lastIt;
     }

     float lastErr = getWeight_Residual(referenceToFrame);








 }

 return 0;
}

/// project the ref 3D point into the new Frame
float cxy::Tracker::getResidual_Buffer(const int& level,
                                      const cxy::TrackRefFrame *const refFrameInput,
                                      const cxy::Frame *const newFrameInput,
                                      const Eigen::Vector3f *const refPoint3DPtrInput,
                                      const Eigen::Vector2f *const refColorVarPtrInput,
                                      Sophus::SE3f &poseInput
                                      )
{
    const auto width = newFrameInput->getWidth(level);
    const auto height = newFrameInput->getHeight(level);
    const auto fx = newFrameInput->getFx(level);
    const auto cx = newFrameInput->getCx(level);
    const auto fy = newFrameInput->getFy(level);
    const auto cy = newFrameInput->getCy(level);

    const auto size = refFrameInput->getNumData(level);
    /// get transformation
    const auto rotationMat = poseInput.rotationMatrix();
    const auto translationVec = poseInput.translation();

    /// init buffers
    int idx=0;
    float sumResUnweighted = 0;
//    bool* isGoodOutBuffer = idxBuf != 0 ? frame->refPixelWasGood() : 0;
    int goodCount = 0;
    int badCount = 0;
    float sumSignedRes = 0;
    float usageCount = 0;
    float sxx=0,syy=0,sx=0,sy=0,sw=0;

    ///pointers
    Eigen::Vector3f const* point3DPtr = refFrameInput->getPoint3D(level);
    Eigen::Vector2f const* refColorVarPtr = refFrameInput->getPointColor_Var(level);
    Eigen::Vector4f const* newFrameGradPtr = newFrameInput->getGradient(level);
    auto isGoodPtr = getMBuf_isPixelGood();
    auto idxPtr = refFrameInput->getPointPosInXYGrid(level);



    for (int ii = 0; ii < size; ++ii, ++idxPtr, ++point3DPtr, ++refColorVarPtr, ++isGoodPtr)
    {
        /// project the ref 3D point into the new Frame
        Eigen::Vector3f projectedPoint = rotationMat*(*point3DPtr) + translationVec;
        float u_new = (projectedPoint[0]/projectedPoint[2])*fx + cx;
        float v_new = (projectedPoint[1]/projectedPoint[2])*fy + cy;

        if ( ! (u_new>1 && u_new<width-2 &&  v_new>1 &&  v_new<height-2 ) )
        {
            continue;
        }

        Eigen::Vector3f resInterp = ImageHelper::getInterpolatedElement43(newFrameGradPtr, u_new, v_new, width);

        // c1 is the resference color. c2 is the interpolated color
        float c1 = affineEstimation_a * (*refColorVarPtr)[0] + affineEstimation_b;
        float c2 = resInterp[2];
        float residual = c1 - c2;

        // use weight function
        auto absResidual = std::fabs(residual);
        auto weight = absResidual < 5.0f ? 1 : 5.0f / absResidual;
        sxx += c1*c1*weight;
        syy += c2*c2*weight;
        sx += c1*weight;
        sy += c2*weight;
        sw += weight;

        bool isGood = residual*residual / (Max_Pixel_Diff_Accept_Throush + Max_Diff_Grad_Multi*(resInterp[0]*resInterp[0] + resInterp[1]*resInterp[1])) < 1;
        *(isGoodPtr) = isGood;

        *(getMBuf_warped_x()+idx) = projectedPoint(0);
        *(getMBuf_warped_y()+idx) = projectedPoint(1);
        *(getMBuf_warped_z()+idx) = projectedPoint(2);

        *(getMBuf_warped_dx()+idx) = fx * resInterp[0];
        *(getMBuf_warped_dy()+idx) = fy * resInterp[1];
        *(getMBuf_warped_residual()+idx) = residual;

        *(getMBuf_d()+idx) = 1.0f / (*point3DPtr)[2];
        *(getMBuf_idepthVar()+idx) = (*refColorVarPtr)[1];


        if(isGood)
        {
            sumResUnweighted += residual*residual;
            sumSignedRes += residual;
            goodCount++;
        }
        else
            badCount++;

        float depthChange = (*point3DPtr)[2] / projectedPoint[2];	// if depth becomes larger: pixel becomes "smaller", hence count it less.
        usageCount += depthChange < 1 ? depthChange : 1;

        /*
        // DEBUG STUFF
        if(plotTrackingIterationInfo || plotResidual)
        {
            // for debug plot only: find x,y again.
            // horribly inefficient, but who cares at this point...
            Eigen::Vector3f point = KLvl * (*refPoint);
            int x = point[0] / point[2] + 0.5f;
            int y = point[1] / point[2] + 0.5f;

            if(plotTrackingIterationInfo)
            {
                setPixelInCvMat(&debugImageOldImageSource,getGrayCvPixel((float)resInterp[2]),u_new+0.5,v_new+0.5,(width/w));
                setPixelInCvMat(&debugImageOldImageWarped,getGrayCvPixel((float)resInterp[2]),x,y,(width/w));
            }
            if(isGood)
                setPixelInCvMat(&debugImageResiduals,getGrayCvPixel(residual+128),x,y,(width/w));
            else
                setPixelInCvMat(&debugImageResiduals,cv::Vec3b(0,0,255),x,y,(width/w));

        }
         */
    }


    buf_warped_size = idx;

    pointUsage = usageCount / (float)size;
    lastGoodCount = goodCount;
    lastBadCount = badCount;
    lastMeanRes = sumSignedRes / goodCount;

    affineEstimation_a_lastIt = sqrtf((syy - sy*sy/sw) / (sxx - sx*sx/sw));
    affineEstimation_b_lastIt = (sy - affineEstimation_a_lastIt*sx)/sw;

    //calcResidualAndBuffers_debugFinish(w);

    return sumResUnweighted / goodCount;


}

float cxy::Tracker::getWeight_Residual(const Sophus::SE3f &refToFrame) {
    const auto &tVec = refToFrame.translation();
    float tx = tVec[0];
    float ty = tVec[1];
    float tz = tVec[2];

    float sumRes = 0;

    /// pointer
    auto warpXPtr = getMBuf_warped_x();
    auto warpYPtr = getMBuf_warped_y();
    auto warpZPtr = getMBuf_warped_z();
    auto depthPtr = getMBuf_d();
    auto warpResiPtr = getMBuf_warped_residual();
    auto warpDxPtr = getMBuf_warped_dx();
    auto warpDyPtr = getMBuf_warped_dy();
    auto idepthVarPtr = getMBuf_idepthVar();

    for (int ii = 0; ii < buf_warped_size; ++ii, ++warpXPtr, ++warpYPtr,
            ++warpZPtr, ++depthPtr, ++warpResiPtr, ++warpDxPtr, ++warpDyPtr, ++idepthVarPtr)
    {

    }

    return 0;
}





