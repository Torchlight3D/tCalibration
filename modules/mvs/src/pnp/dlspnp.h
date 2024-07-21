#pragma once

#include <Eigen/Geometry>

namespace tl {

// Brief:
// Computes the camera pose with PnP.
//
// This method is extremely scalable and highly accurate for the PnP problem. A
// minimum of 3 points are required, but there is no maximum number of points
// allowed as this is a least-squared approach.
//
// The general approach is to first rewrite the reprojection constraint (i.e.,
// cost function) such that all unknowns appear linearly in terms of the
// rotation parameters (which are 3 parameters in the Cayley-Gibss-Rodriguez
// formulation). Then we create a system of equations from the jacobian of the
// cost function, and solve these equations via a Macaulay matrix to obtain the
// roots (i.e., the 3 parameters of rotation). The translation can then be
// obtained through back-substitution.
//
// Note:
// 1. Theoretically, up to 27 solutions may be returned, but in practice only 4
// real solutions arise and in almost all cases where n >= 6 there is only one
// solution which places the observed points in front of the camera.
//
// Ref:
// "A Direct Least-Squares (DLS) Method for PnP" by Joel Hesch and Stergios
// Roumeliotis.
//
bool DLSPnp(const std::vector<Eigen::Vector2d>& imagePoints,
            const std::vector<Eigen::Vector3d>& worldPoints,
            std::vector<Eigen::Quaterniond>& rotations,
            std::vector<Eigen::Vector3d>& translations);

} // namespace tl
