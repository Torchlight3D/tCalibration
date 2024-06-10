#pragma once

#include <Eigen/Core>

#include "cameraintrinsics.h"

namespace ceres {
class CostFunction;
}

namespace tl {

// TODO: Use auto registration
ceres::CostFunction* createReprojectionErrorCostFunction(
    CameraIntrinsicsType type, const Eigen::Vector2d& feature);

} // namespace tl
