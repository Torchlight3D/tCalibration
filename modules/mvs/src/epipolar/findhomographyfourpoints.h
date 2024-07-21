#pragma once

#include <Eigen/Core>

namespace tl {

// Brief:
// Normalized DLT method to compute the homography H that maps image points in
// image_1 to image_2 via x' = Hx (where x is in image 1 and x' is in image
// 2). Note this is only valid for planar motion (i.e., rotation-only movement,
// or tracking points on a plane). The DLT algorithm implemented is from
// Algorithm 4.2 in Hartley and Zisserman (page 109).
// Ref:
//
bool FourPointHomography(const std::vector<Eigen::Vector2d>& imagePoints1,
                         const std::vector<Eigen::Vector2d>& imagePoints2,
                         Eigen::Matrix3d* homography);

bool FindHomographyFourPoint(const std::vector<Eigen::Vector2d>& imagePoints1,
                             const std::vector<Eigen::Vector2d>& imagePoints2,
                             Eigen::Matrix3d* homography);

} // namespace tl
