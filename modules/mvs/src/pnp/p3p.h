#pragma once

#include <Eigen/Core>

namespace tl {

// Brief:
// Computes camera pose with 3 pairs of calibrated 2D-3D correspondences
//
// Requirement:
// 1. Image points should be calibrated with the camera intrinsics.
// 2. At least three correspondences are required. In fact, only the first three
// point pairs are used in calculation.
//
// NOTE:
// 1. Up to 4 solutions. Need neccessary post-processing
//
// Ref:
// "A Novel Parameterization of the Perspective-Three-Point Problem for a direct
// computation of Absolute Camera position and Orientation" by Kneip et. al.
bool P3P(const std::vector<Eigen::Vector2d>& imagePoints,
         const std::vector<Eigen::Vector3d>& objectPoints,
         std::vector<Eigen::Matrix3d>& rotations,
         std::vector<Eigen::Vector3d>& translations);

} // namespace tl
