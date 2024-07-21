#pragma once

#include <Eigen/Core>

namespace tl {

// Brief:
// Computes the fundamental matrix from 8 (or more) image correspondences
// according to the normalized 8 point algorithm. Image points are first
// normalized by a translation and scale, and the fundamental matrix is computed
// from the singular vector corresponding to the smallest singular vector of the
// stacked epipolar constraints. The estimated fundamental matrix is the
// computed fundamental matrix with the normalization transformation undone.
//
// Inputs:
//   imagePoints1: image points from first image (8 or more).
//   imagePoints1: image points from second image (8 or more).
// Outputs:
//   fmatrix: the estimated fundamental matrix such that
//                    x2' * F * x1 = 0
// Return:
//
//
// Ref:
// MVG. algorithm 11.1
bool FindFundamentalEightPoints(
    const std::vector<Eigen::Vector2d>& imagePoints1,
    const std::vector<Eigen::Vector2d>& imagePoints2, Eigen::Matrix3d* fmatrix);

// Remove later
bool NormalizedEightPointFundamentalMatrix(
    const std::vector<Eigen::Vector2d>& imagePoints1,
    const std::vector<Eigen::Vector2d>& imagePoints2, Eigen::Matrix3d* fmatrix);

} // namespace tl
