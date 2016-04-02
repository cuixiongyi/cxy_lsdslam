//
// Created by xiongyi on 3/31/16.
//

#ifndef CXY_LSDSLAM_DEBUGUTILITY_H
#define CXY_LSDSLAM_DEBUGUTILITY_H


#include <opencv2/core/hal/interface.h>

namespace cxy
{
    class DebugUtility {

    public:
        static void DisplayImage(int width, int height, int type, void* data, std::string windowName);

    };

}


#endif //CXY_LSDSLAM_DEBUGUTILITY_H
