#pragma once

#include <Eigen/Geometry>

namespace tl {

// Brief:
// Computes the camera pose using the Perspective N-point method from
//
// Requirement:
// 1. At least three correspondences are required.
//
// Ref:
// "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point
// Problem" by G. Terzakis and M. Lourakis
bool SQPnP(const std::vector<Eigen::Vector2d>& imagePoints,
           const std::vector<Eigen::Vector3d>& worldPoints,
           std::vector<Eigen::Quaterniond>& rotations,
           std::vector<Eigen::Vector3d>& translations);

} // namespace tl
