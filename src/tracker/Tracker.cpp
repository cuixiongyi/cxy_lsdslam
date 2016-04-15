//
// Created by xiongyi on 3/28/16.
//

#include <src/utility/ImageHelper.h>
#include <src/Optimization/NormalEquationLeastSquare.h>
#include "src/tracker/Tracker.h"

namespace cxy
{





cxy::Tracker::Tracker(const int& width, const int& height)
{

    // load Parameters
    Const_Max_Pixel_Diff_Accept_Throush = ParameterServer::getParameter<float>("Const_Max_Pixel_Diff_Accept_Throush");
    Const_Max_Diff_Grad_Multi = ParameterServer::getParameter<float>("Const_Max_Diff_Grad_Multi");
    Const_IDepthVarianceWeight  = ParameterServer::getParameter<float>("Const_IDepthVarianceWeight");
    Const_ImageColorVariance = ParameterServer::getParameter<float>("Const_ImageColorVariance");
    Const_Huber_D = ParameterServer::getParameter<float>("Const_Huber_D");
    Const_LambdaSuccessFac = ParameterServer::getParameter<float>("LambdaSuccessFac");
    Const_LambdaFailFactor = ParameterServer::getParameter<float>("LambdaFailFactor");

    auto lambdaInitialLevelConfig = ParameterServer::getParameterNode("LambdaInitialLevel");
    auto IterationNumConfig = ParameterServer::getParameterNode("IterationNumLevel");
    auto convergenceEpsConfig = ParameterServer::getParameterNode("ConvergenceEps");
    auto stepSizeMinConfig = ParameterServer::getParameterNode("StepSizeMin");
    for (int ii = 0; ii < lambdaInitialLevelConfig.size(); ++ii)
    {
        Const_LambdaInitialLevel.push_back(lambdaInitialLevelConfig[ii].as<float>());
        Const_IterationNumLevel.push_back(IterationNumConfig[ii].as<int>());
        Const_ConvergenceEps.push_back(convergenceEpsConfig[ii].as<float>());
        Const_StepSizeMin.push_back(stepSizeMinConfig[ii].as<float>());
    }
    assert(Const_LambdaInitialLevel.size() == MAX_PYRAMID_LEVEL);

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



    SE3  cxy::Tracker::track_NoDepth(const cxy::TrackRefFrame *const refFrameInput,
                                cxy::Frame *const newFrameInput,
                                const Sophus::SE3f &frameToRefInput)
{

    mRefTrackFrame = refFrameInput;
    mNewTrackFrame = newFrameInput;

    Sophus::SE3f refToFramePose = frameToRefInput.inverse();
    NormalEquationLeastSquare ls;
    float lastIterResidual = 0.f;

    //// loop over levels
 for (int ll = MAX_PYRAMID_LEVEL - 1; ll >= 0; ++ll)
 {
     const auto width = newFrameInput->getWidth(ll);
     const auto height = newFrameInput->getHeight(ll);
     const auto size = width*height;
     //// init mBuf_isPixelGood
     for (int ii = 0; ii < size; ++ii)
         mBuf_isPixelGood[ii] = false;
     affineEstimation_a = 1; affineEstimation_b = 0;



     getResidual_Buffer(ll,
                        refFrameInput,
                        newFrameInput,
                        refFrameInput->getPoint3D(ll),
                        refFrameInput->getPointColor_Var(ll),
                        refToFramePose);
     if(Const_UseAffineLightningEstimation)
     {
         affineEstimation_a = affineEstimation_a_lastIt;
         affineEstimation_b = affineEstimation_b_lastIt;
     }

     float lastErr = getWeight_Residual(refToFramePose);

     float LM_lambda = Const_LambdaInitialLevel[ll];

     for(int iteration=0; iteration < Const_IterationNumLevel[ll]; iteration++)
     {
//        getJacobian_Update();
         getJacobian_Update(ls);
         unsigned int incTry=0;

         while(true)
         {
             // solve LS system with current lambda
             Vector6f b = -ls.b;
             Matrix66f A = ls.A;
             ls.setLambda(LM_lambda);
             Vector6f inc = A.ldlt().solve(b);
             ++incTry;


             // apply increment. pretty sure this way round is correct, but hard to test.
             Sophus::SE3f newReferenceToFrame = Sophus::SE3f::exp((inc)) * refToFramePose;
             //Sophus::SE3f new_referenceToFrame = referenceToFrame * Sophus::SE3f::exp((inc));

             getResidual_Buffer(ll,
                                refFrameInput,
                                newFrameInput,
                                refFrameInput->getPoint3D(ll),
                                refFrameInput->getPointColor_Var(ll),
                                newReferenceToFrame);
             /*
             if(buf_warped_size < MIN_GOODPERALL_PIXEL_ABSMIN* (width>>lvl)*(height>>lvl))
             {
                 diverged = true;
                 trackingWasGood = false;
                 return SE3();
             }
             */

             float error = getWeight_Residual(newReferenceToFrame);

             ROS_INFO("          lambda: %f     error: %f", LM_lambda, error);
             // accept inc?
             if(error < lastErr)
             {
                 incTry++;
                 // accept inc
                 refToFramePose = newReferenceToFrame;
//             if(useAffineLightningEstimation)
                 if (0)
                 {
                     affineEstimation_a = affineEstimation_a_lastIt;
                     affineEstimation_b = affineEstimation_b_lastIt;
                 }

/*
             if(enablePrintDebugInfo && printTrackingIterationInfo)
             {
                 // debug output
                 printf("(%d-%d): ACCEPTED increment of %f with lambda %.1f, residual: %f -> %f\n",
                        lvl,iteration, sqrt(inc.dot(inc)), LM_lambda, lastErr, error);

                 printf("         p=%.4f %.4f %.4f %.4f %.4f %.4f\n",
                        referenceToFrame.log()[0],referenceToFrame.log()[1],referenceToFrame.log()[2],
                        referenceToFrame.log()[3],referenceToFrame.log()[4],referenceToFrame.log()[5]);
             }
*/
                 // converged?
                 if(error / lastErr > Const_ConvergenceEps[ll])
                 {
//                 if(enablePrintDebugInfo && printTrackingIterationInfo)
                     {
                         printf("(%d-%d): FINISHED pyramid level (last residual reduction too small).\n",
                                ll,iteration);
                     }
                     iteration = Const_IterationNumLevel[ll];
                 }

                 lastIterResidual = lastErr = error;


                 if(LM_lambda <= 0.2)
                     LM_lambda = 0;
                 else
                     LM_lambda *= Const_LambdaSuccessFac;

                 break;
             }
             else
             {
                 /*
                 if(enablePrintDebugInfo && printTrackingIterationInfo)
                 {
                     printf("(%d-%d): REJECTED increment of %f with lambda %.1f, (residual: %f -> %f)\n",
                            lvl,iteration, sqrt(inc.dot(inc)), LM_lambda, lastErr, error);
                 }
                 */

                 if(!(inc.dot(inc) > Const_StepSizeMin[ll]))
                 {

//                 if(enablePrintDebugInfo && printTrackingIterationInfo)
                     {
                         printf("(%d-%d): FINISHED pyramid level (stepsize too small).\n",
                                ll,iteration);
                     }
                     iteration = Const_IterationNumLevel[ll];
                     break;
                 }

                 if(LM_lambda == 0)
                     LM_lambda = 0.2;
                 else
                     LM_lambda *= std::pow(Const_LambdaFailFactor, incTry);
             }


         }
         ROS_INFO("    iteration: %d      lambda: %f     error: %f", iteration, LM_lambda, lastErr);
     } /// Iteration

 }// pyramid


    lastResidual = lastIterResidual;

    /*
    trackingWasGood = !diverged
                      && lastGoodCount / (frame->width(SE3TRACKING_MIN_LEVEL)*frame->height(SE3TRACKING_MIN_LEVEL)) > MIN_GOODPERALL_PIXEL
                      && lastGoodCount / (lastGoodCount + lastBadCount) > MIN_GOODPERGOODBAD_PIXEL;
    if(trackingWasGood)
        reference->keyframe->numFramesTrackedOnThis++;
*/

    newFrameInput->mInitialTrackedResidual = lastResidual / pointUsage;
    newFrameInput->pose->thisToParent_raw = ImageHelper::sim3FromSE3(refToFramePose.inverse().cast<float>(),1);
    newFrameInput->pose->trackingParent = refFrameInput->getFrame()->pose.get();
    return refToFramePose.inverse();
// return 0;
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

        bool isGood = residual*residual / (Const_Max_Pixel_Diff_Accept_Throush + Const_Max_Diff_Grad_Multi*(resInterp[0]*resInterp[0] + resInterp[1]*resInterp[1])) < 1;
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
//    DebugUtility::PublishPointCloud()

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
    auto weightP_Ptr = getMBuf_weight_p();

    for (int ii = 0; ii < buf_warped_size; ++ii, ++warpXPtr, ++warpYPtr,
            ++warpZPtr, ++depthPtr, ++warpResiPtr, ++warpDxPtr, ++warpDyPtr, ++idepthVarPtr, ++weightP_Ptr)
    {

        float px = *(warpXPtr);	// x'
        float py = *(warpYPtr);	// y'
        float pz = *(warpZPtr);	// z'
        float d = *(depthPtr);	// d
        float rp = *(warpResiPtr); // r_p
        float gx = *(warpDxPtr);	// \delta_x ii
        float gy = *(warpDyPtr);  // \delta_y I
        float s = Const_IDepthVarianceWeight* *(idepthVarPtr);	// \sigma_d^2


        // calc dw/dd (first 2 components):
        float g0 = (tx * pz - tz * px) / (pz*pz*d);
        float g1 = (ty * pz - tz * py) / (pz*pz*d);


        // calc w_p
        float drpdd = gx * g0 + gy * g1;	// ommitting the minus
        float w_p = 1.0f / ((Const_ImageColorVariance) + s * drpdd * drpdd);

        float weighted_rp = fabs(rp*sqrtf(w_p));

        float wh = fabs(weighted_rp < (Const_Huber_D/2) ? 1 : (Const_Huber_D/2) / weighted_rp);

        sumRes += wh * w_p * rp*rp;


        *(weightP_Ptr) = wh * w_p;
    }

    return sumRes / buf_warped_size;
}

cxy::Vector6f cxy::Tracker::getJacobian_Update(NormalEquationLeastSquare& ls)
{
    /// pointer
    auto warpXPtr = getMBuf_warped_x();
    auto warpYPtr = getMBuf_warped_y();
    auto warpZPtr = getMBuf_warped_z();
    auto depthPtr = getMBuf_d();
    auto warpResiPtr = getMBuf_warped_residual();
    auto warpDxPtr = getMBuf_warped_dx();
    auto warpDyPtr = getMBuf_warped_dy();
    auto idepthVarPtr = getMBuf_idepthVar();
    auto weightP_Ptr = getMBuf_weight_p();
    ls.initialize(buf_warped_size);
    for(int i=0;i<buf_warped_size;i++)
    {
        float px = *(warpXPtr+i);
        float py = *(warpYPtr+i);
        float pz = *(warpZPtr+i);
        float r =  *(warpResiPtr+i);
        float gx = *(warpDxPtr+i);
        float gy = *(warpDyPtr+i);
        // step 3 + step 5 comp 6d error vector

        float z = 1.0f / pz;
        float z_sqr = 1.0f / (pz*pz);
        Vector6f v;
        v[0] = z*gx + 0;
        v[1] = 0 +         z*gy;
        v[2] = (-px * z_sqr) * gx +
               (-py * z_sqr) * gy;
        v[3] = (-px * py * z_sqr) * gx +
               (-(1.0 + py * py * z_sqr)) * gy;
        v[4] = (1.0 + px * px * z_sqr) * gx +
               (px * py * z_sqr) * gy;
        v[5] = (-py * z) * gx +
               (px * z) * gy;

        // step 6: integrate into A and b:
        ls.update(v, r, *(weightP_Ptr+i));
    }
    Vector6f result;

    // solve ls
    ls.finish();
    ls.solve(result);

    return result;
}




}




