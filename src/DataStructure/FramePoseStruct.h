//
// Created by xiongyi on 4/10/16.
//

#ifndef CXY_LSDSLAM_FRAMEPOSESTRUCT_H
#define CXY_LSDSLAM_FRAMEPOSESTRUCT_H

#include "DataTypeDeclearation.h"
namespace cxy
{
	class VertexSim3;
	class Frame;
    class FramePoseStruct
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		FramePoseStruct(Frame*const frame);
		virtual ~FramePoseStruct();

		// parent, the frame originally tracked on. never changes.
		FramePoseStruct* trackingParent;

		// set initially as tracking result (then it's a SE(3)),
		// and is changed only once, when the frame becomes a KF (->rescale).
		Sim3 thisToParent_raw;


		int frameID;
		Frame* frame;


		// whether this poseStruct is registered in the Graph. if true MEMORY WILL BE HANDLED BY GRAPH
		bool isRegisteredToGraph;

		// whether pose is optimized (true only for KF, after first applyPoseGraphOptResult())
		bool isOptimized;

		// true as soon as the vertex is added to the g2o graph.
		bool isInGraph;

		// graphVertex (if the frame has one, i.e. is a KF and has been added to the graph, otherwise 0).
		VertexSim3* graphVertex;

		void setPoseGraphOptResult(Sim3 camToWorld);
		void applyPoseGraphOptResult();
		Sim3 getCamToWorld(int recursionDepth = 0);
		void invalidateCache();
	private:
		int cacheValidFor;
		static int cacheValidCounter;

		// absolute position (camToWorld).
		// can change when optimization offset is merged.
		Sim3 camToWorld;

		// new, optimized absolute position. is added on mergeOptimization.
		Sim3 camToWorld_new;

		// whether camToWorld_new is newer than camToWorld
		bool hasUnmergedPose;
	};
}


#endif //CXY_LSDSLAM_FRAMEPOSESTRUCT_H
