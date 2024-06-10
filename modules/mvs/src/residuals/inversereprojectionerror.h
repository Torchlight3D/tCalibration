#pragma once

#include <Eigen/Core>

#include <tCamera/Camera>
#include <tMvs/Feature>

namespace ceres {
class CostFunction;
}

namespace tl {

// TODO: Use auto registration
ceres::CostFunction* createInvReprojectionPoseErrorCostFunction(
    CameraIntrinsicsType type, const Feature& feature,
    const Eigen::Vector3d& ref_bearing);

} // namespace tl
