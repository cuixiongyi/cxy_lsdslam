//
// Created by xiongyi on 4/3/16.
//

#include "TrackRefFrame.h"

cxy::TrackRefFrame::TrackRefFrame(const cxy::Frame *const refFrameInput)
: mFrame(refFrameInput)
{
    for (int ii = 0; ii < MAX_PYRAMID_LEVEL; ++ii) {
        numData.push_back(0);
        makePointCloud(ii);
    }

}

void cxy::TrackRefFrame::makePointCloud(int level) {
    assert(mFrame != nullptr);

    if(numData[level] > 0)
        return;	// already exists.

    const int& width = mFrame->getWidth(level);
    const int& height = mFrame->getHeight(level);
    const auto size = width*height;
    float fxInv = mFrame->getFxInv(level);
    float fyInv = mFrame->getFyInv(level);
    float cxInv = mFrame->getCxInv(level);
    float cyInv = mFrame->getCyInv(level);

    float const*  idepthSourcePtr = mFrame->getIdepth(level);
    float const* idepthVarSourcePtr = mFrame->getIdepthVar(level);
    float const* colorSourcePtr = mFrame->getImage(level);
    Eigen::Vector4f const*  gradSourcePtr = mFrame->getGradient(level);

    point3D.push_back(std::move(MemoryManager::ArrayPointer_Allocator<Eigen::Vector3f>(size)));
    pointGrad.push_back(std::move(MemoryManager::ArrayPointer_Allocator<Eigen::Vector2f>(size)));
    pointColor_Var.push_back(std::move(MemoryManager::ArrayPointer_Allocator<Eigen::Vector2f>(size)));
    pointPosInXYGrid.push_back(std::move(MemoryManager::ArrayPointer_Allocator<unsigned int>(size)));

    Eigen::Vector3f*  point3DPtr = getPoint3D(level);
    auto* pointIdxPtr = getPointPosInXYGrid(level);
    Eigen::Vector2f* gradDataPT = getPointGrad(level);
    Eigen::Vector2f* colorAndVarDataPT = getPointColor_Var(level);

    for(int x=1; x<width-1; x++)
        for(int y=1; y<height-1; y++)
        {
            int idx = x + y*width;

            if(idepthVarSourcePtr[idx] <= 0 || idepthSourcePtr[idx] == 0) continue;

            *point3DPtr = (1.0f / idepthSourcePtr[idx]) * Eigen::Vector3f(fxInv*x+cxInv,fyInv*y+cyInv,1);
            *gradDataPT = gradSourcePtr[idx].head<2>();
            *colorAndVarDataPT = Eigen::Vector2f(colorSourcePtr[idx], idepthVarSourcePtr[idx]);
            *pointIdxPtr = idx;

            point3DPtr++;
            gradDataPT++;
            colorAndVarDataPT++;
            pointIdxPtr++;
        }

    numData.push_back(point3DPtr-getPoint3D(level));
}



