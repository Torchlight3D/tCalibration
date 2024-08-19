#pragma once

#include <Eigen/Core>

namespace tl {

bool MLPnP(const std::vector<Eigen::Vector2d>& norm_feature_points,
           const std::vector<Eigen::Matrix3d>& feature_covariances,
           const std::vector<Eigen::Vector3d>& world_points,
           Eigen::Matrix3d* solution_rotations,
           Eigen::Vector3d* solution_translations);

}
