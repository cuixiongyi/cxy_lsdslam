//
// Created by xiongyi on 4/3/16.
//

#ifndef CXY_LSDSLAM_TRACKFRAME_H
#define CXY_LSDSLAM_TRACKFRAME_H

#include <src/DataStructure/Frame.h>
#include "utility/MemoryManager.h"
#include "utility/MacroUtility.h"


namespace cxy
{
    class TrackRefFrame {

    public:
        TrackRefFrame(Frame const*const refFrameInput);
        inline Eigen::Vector3f const*const getPoint3D(int level) const {return point3D[level].get();};
        inline Eigen::Vector3f* getPoint3D(int level) {return point3D[level].get();};

        inline Eigen::Vector2f const*const getPointGrad(int level) const {return pointGrad[level].get();};
        inline Eigen::Vector2f*  getPointGrad(int level)  {return pointGrad[level].get();};

        inline Eigen::Vector2f const*const getPointColor_Var(int level) const {return pointColor_Var[level].get();};
        inline Eigen::Vector2f* getPointColor_Var(int level)  {return pointColor_Var[level].get();};

        inline unsigned int const*const getPointPosInXYGrid(int level) const {return pointPosInXYGrid[level].get();};
        inline unsigned int* getPointPosInXYGrid(int level) {return pointPosInXYGrid[level].get();};

        inline const unsigned int& getNumData(int level) const { return numData[level];}
    private:
        Frame const*const mFrame;

        ///3DData
        void makePointCloud(int level);
        ///posData
        ArrayPointer_Vector<Eigen::Vector3f> point3D;	// (x,y,z)
        ///gradData
        ArrayPointer_Vector<Eigen::Vector2f> pointGrad;	// (dx, dy)
        ///colorAndVarData
        ArrayPointer_Vector<Eigen::Vector2f> pointColor_Var;	// (I, Var)

        ArrayPointer_Vector<unsigned int> pointPosInXYGrid;	// x + y*mWidth
        std::vector<unsigned int> numData;

    };


}


#endif //CXY_LSDSLAM_TRACKFRAME_H
