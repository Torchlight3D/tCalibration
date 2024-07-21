#pragma once

#include <Eigen/Core>

namespace tl {

// Brief:
// Computes the relative pose between two cameras using 5 corresponding
// points. Algorithm is implemented based on  The relative pose is computed
// such that y * E * x = 0, where E = t_x * R and t_x is the cross product
// matrix of t. This implementation is proven to be more stable than the fast 5
// pt. algorithm by Nister.
//
// Inputs:
//   imagePoints1: Location of features on the image plane of image 1.
//   imagePoints2: Location of features on the image plane of image 2.
// Outputs:
//   ematrices: Estimated essential matrices
// Return:
//   true if a valid solution was found.
//
// Ref:
// [1] "Recent developments on direct relative orientation" by H.Stewénius,
// C.Engels, and D.Nistér (ISPRS Journal of Photogrammetry and Remote Sensing,
// 2006)
// [2] "An Efficient Solution to the Five-Point Relative Pose Problem" from
// Nister
//
// NOTE: At least 5 points must be supplied, but a non-minimal estimate
// will be computed if more than five are supplied.
bool FindEssentialFivePoints(const std::vector<Eigen::Vector2d>& imagePoints1,
                             const std::vector<Eigen::Vector2d>& imagePoints2,
                             std::vector<Eigen::Matrix3d>* ematrices);

// Remove later
bool FivePointRelativePose(const std::vector<Eigen::Vector2d>& imagePoints1,
                           const std::vector<Eigen::Vector2d>& imagePoints2,
                           std::vector<Eigen::Matrix3d>* ematrices);

} // namespace tl
