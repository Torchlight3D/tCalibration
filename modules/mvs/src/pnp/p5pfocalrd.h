#pragma once

#include <Eigen/Core>

namespace tl {

// Description: Compute the absolute pose, focal length, and radial distortion
// of a camera using five 3D-to-2D correspondences from the ICCV paper: "Real
// time solution to the absolute pose problem with unknown radial distortion and
// focal length" by Kukelova et. al.
//
// The method solves for the projection matrix (up to scale) by using a cross
// product constraint on the standard projection equation. This allows for
// simple solution to the first two rows of the projection matrix, and the third
// row (which contains the focal length and distortion parameters) can then be
// solved with SVD on the remaining constraint equations from the first row of
// the projection matrix. See the paper for more details.
//
// Input:
//   feature_positions: feature positions (must be 5 features).
//   world_points: 5 3-vectors with corresponding 3D world points
//   num_radial_distortion_params: The number of radial distortion paramters to
//     solve for. Must be 1, 2, or 3. Experiments by cmsweeney showed that the
//     solution method is unstable when no radial distortion is solved for.
//   solutions: Camera projection matrices (that encapsulate focal
//     length) and radial distortion parameters. Thes are only up to scale.
// Output: true if success, false if not.

bool FivePointFocalLengthRadialDistortion(
    const std::vector<Eigen::Vector2d>& feature_positions,
    const std::vector<Eigen::Vector3d>& world_points,
    int num_radial_distortion_params,
    std::vector<Eigen::Matrix<double, 3, 4>>* projection_matrices,
    std::vector<std::vector<double>>* radial_distortions);

} // namespace tl
