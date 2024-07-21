#pragma once

#include <Eigen/Core>

namespace tl {

struct Feature2D2D;

// Brief:
// Using known relative rotations, optimize the relative position that minimizes
// the epipolar constraint for all 2D-2D corrs.
//                      x2' * [t]_x * R * x1 = 0
// NOTE: the position is -R' * t and the rotations correspond to the absolute
// orientations of cameras 1 and 2.
//
// Ref:
// "Robust Camera Location Estimation by Convex Programming" by Onur Ozyesil and
// Amit Singer (CVPR 2015).
bool OptimizeRelativePositionWithKnownRotation(
    const std::vector<Feature2D2D>& corrs, const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2, Eigen::Vector3d* relative_position);

} // namespace tl
