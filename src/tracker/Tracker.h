//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_TRACKER_H
#define CXY_LSDSLAM_TRACKER_H

#include "TrackFrame.h"
#include "sophus/se3.hpp"

namespace cxy
{
    class Tracker {
        int track_NoDepth(TrackFrame const*const refTrackFrameInput,
                            Frame const*const trackFrameInput,
                            const Sophus::SE3f& initPose);
        int track_WithDepth();

    private:
        TrackFrame const* mRefTrackFrame = nullptr;
        Frame const* mTrackFrame = nullptr;
    };


}


#endif //CXY_LSDSLAM_TRACKER_H
