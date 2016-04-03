//
// Created by xiongyi on 4/3/16.
//

#include "TrackFrame.h"

cxy::TrackFrame::TrackFrame(const cxy::Frame *const refFrameInput)
: mFrame(refFrameInput){

}

void cxy::TrackFrame::makePointCloud(int level) {
    assert(mFrame != nullptr);

    if(numData[level] > 0)
        return;	// already exists.

    const int& width = mFrame->getWidth(level);
    const int& height = mFrame->getHeight(level);
    const auto size = width*height;
    float fxInvLevel = mFrame->getFxInv(level);
    float fyInvLevel = mFrame->getFyInv(level);
    float cxInvLevel = mFrame->getCxInv(level);
    float cyInvLevel = mFrame->getCyInv(level);

    float const*const  pyrIdepthSource = mFrame->getIdepth(level);;
    float const*const pyrIdepthVarSource = mFrame->getIdepthVar(level);
    float const*const pyrColorSource = mFrame->getImage(level);
    Eigen::Vector4f const*const  pyrGradSource = mFrame->getGradient(level);

    point3D.push_back(std::move(MemoryManager::ArrayPointer_Allocator<Eigen::Vector3f>(size)));
    pointGrad.push_back(std::move(MemoryManager::ArrayPointer_Allocator<Eigen::Vector2f>(size)));
    pointColor_Var.push_back(std::move(MemoryManager::ArrayPointer_Allocator<Eigen::Vector2f>(size)));
    pointPosInXYGrid.push_back(std::move(MemoryManager::ArrayPointer_Allocator<int>(size)));

    Eigen::Vector3f const*const point3DPtr = getPoint3D(level);
    int const*const pointIdxPtr = getPointPosInXYGrid(level);
    Eigen::Vector2f const*const gradDataPT = getPointGrad(level);
    Eigen::Vector2f const*const colorAndVarDataPT = getPointColor_Var(level);

    for(int x=1; x<width-1; x++)
        for(int y=1; y<height-1; y++)
        {
            int idx = x + y*width;

            if(pyrIdepthVarSource[idx] <= 0 || pyrIdepthSource[idx] == 0) continue;

            
            *posDataPT = (1.0f / pyrIdepthSource[idx]) * Eigen::Vector3f(fxInvLevel*x+cxInvLevel,fyInvLevel*y+cyInvLevel,1);
            *gradDataPT = pyrGradSource[idx].head<2>();
            *colorAndVarDataPT = Eigen::Vector2f(pyrColorSource[idx], pyrIdepthVarSource[idx]);
            *idxPT = idx;

            posDataPT++;
            gradDataPT++;
            colorAndVarDataPT++;
            idxPT++;
        }

    numData[level] = posDataPT - posData[level];
}



