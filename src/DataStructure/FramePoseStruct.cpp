//
// Created by xiongyi on 4/10/16.
//

#include "FramePoseStruct.h"
#include "Frame.h"

namespace cxy
{

    int FramePoseStruct::cacheValidCounter = 0;

    int privateFramePoseStructAllocCount = 0;

    FramePoseStruct::FramePoseStruct(Frame*const frame)
    {
        cacheValidFor = -1;
        isOptimized = false;
        thisToParent_raw = camToWorld = camToWorld_new = Sim3();
        this->frame = frame;
        frameID = frame->getFrameid();
        trackingParent = 0;
        isRegisteredToGraph = false;
        hasUnmergedPose = false;
        isInGraph = false;

        this->graphVertex = nullptr;

        privateFramePoseStructAllocCount++;
//        if(enablePrintDebugInfo && printMemoryDebugInfo)
//            printf("ALLOCATED pose %d, now there are %d\n", frameID, privateFramePoseStructAllocCount);
    }

    FramePoseStruct::~FramePoseStruct()
    {
        privateFramePoseStructAllocCount--;
//        if(enablePrintDebugInfo && printMemoryDebugInfo)
//            printf("DELETED pose %d, now there are %d\n", frameID, privateFramePoseStructAllocCount);
    }

    void FramePoseStruct::setPoseGraphOptResult(Sim3 camToWorld)
    {
        if(!isInGraph)
            return;


        camToWorld_new = camToWorld;
        hasUnmergedPose = true;
    }

    void FramePoseStruct::applyPoseGraphOptResult()
    {
        if(!hasUnmergedPose)
            return;


        camToWorld = camToWorld_new;
        isOptimized = true;
        hasUnmergedPose = false;
        cacheValidCounter++;
    }
    void FramePoseStruct::invalidateCache()
    {
        cacheValidFor = -1;
    }
    Sim3 FramePoseStruct::getCamToWorld(int recursionDepth)
    {
        // prevent stack overflow
        assert(recursionDepth < 5000);

        // if the node is in the graph, it's absolute pose is only changed by optimization.
        if(isOptimized) return camToWorld;


        // return chached pose, if still valid.
        if(cacheValidFor == cacheValidCounter)
            return camToWorld;

        // return id if there is no parent (very first frame)
        if(trackingParent == nullptr)
            return camToWorld = Sim3();

        // abs. pose is computed from the parent's abs. pose, and cached.
        cacheValidFor = cacheValidCounter;

        return camToWorld = trackingParent->getCamToWorld(recursionDepth+1) * thisToParent_raw;
    }

}
