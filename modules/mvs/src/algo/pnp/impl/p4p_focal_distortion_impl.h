#pragma once

#include <vector>
#include <Eigen/Core>

namespace tl {

void FourPointsPoseFocalLengthRadialDistortionSolver(
    const Eigen::Matrix<double, 64, 1>& data,
    std::vector<Eigen::Matrix<double, 5, 1>>* solutions);

} // namespace tl
