#pragma once

#include <Eigen/Core>

#include <tCamera/Camera>
#include <tMvs/Feature>

namespace ceres {
class CostFunction;
}

namespace tl {

ceres::CostFunction* createInvReprojectionPoseErrorCostFunction(
    CameraIntrinsics::Type type, const Feature& feature,
    const Eigen::Vector3d& ref_bearing);

}
