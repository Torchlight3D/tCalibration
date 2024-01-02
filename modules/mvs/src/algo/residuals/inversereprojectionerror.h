#pragma once

#include <Eigen/Core>

#include <AxCamera/Camera>
#include <AxMVS/Feature>

namespace ceres {
class CostFunction;
}

namespace thoht {

ceres::CostFunction* createInvReprojectionPoseErrorCostFunction(
    CameraIntrinsics::Type type, const Feature& feature,
    const Eigen::Vector3d& ref_bearing);

}
