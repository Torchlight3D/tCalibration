#pragma once

#include <tMath/Eigen/Types>

namespace tl {

// Brief:
// Computes the absolute pose and focal length of a camera from 4 pairs of 2D-3D
// correspondences.
//
// Explanation:
// The solution involves computing a grobner basis based on a unique constraint
// of the focal length and pose reprojection.
//
// Reference paper:
// "A general solution to the P4P problem for camera with unknown focal length"
// by Bujnak et al.
//
// Input:
//     imagePoints: 4 vectors with image positions.
//     worldPoints: 4 3-vectors with corresponding 3D world points (each entry
//     is a point)
// Output:
//     solutions: Camera projection matrices (that encapsulate focal length)
// Return:
//     int: number of solutions if correct execution -1 if invalid.

int FourPointPoseAndFocalLength(const std::vector<Eigen::Vector2d>& imagePoints,
                                const std::vector<Eigen::Vector3d>& worldPoints,
                                std::vector<Matrix34d>& projectionMatrices);

} // namespace tl
