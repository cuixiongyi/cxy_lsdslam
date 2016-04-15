//
// Created by xiongyi on 4/8/16.
//

#ifndef CXY_LSDSLAM_DATATYPE_H
#define CXY_LSDSLAM_DATATYPE_H

#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>
#include "Eigen/Eigen"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
namespace cxy
{
    using Vector6f = Eigen::Matrix<float, 6, 1>;
    using Matrix66f = Eigen::Matrix<float, 6, 6>;
    using Sim3 = Sophus::Sim3f;
    using SE3 = Sophus::SE3f;
    using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
}
#endif //CXY_LSDSLAM_DATATYPE_H
