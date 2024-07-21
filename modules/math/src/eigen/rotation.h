#pragma once

#include <optional>

#include <Eigen/Core>

namespace tl {

Eigen::MatrixXd ProjectToSOd(const Eigen::MatrixXd& M);

// Computes R_ij = R_j * R_i' (with noise) in rotation vector
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2,
    const std::optional<double>& noise = {});

Eigen::Vector3d RelativeTranslationFromTwoPositions(
    const Eigen::Vector3d& position1, const Eigen::Vector3d& position2,
    const Eigen::Vector3d& rotation1, const std::optional<double>& noise = {});

Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rvec1,
                                  const Eigen::Vector3d& rvec2);

// Compute R_j = R_ij * R_i in rotation vector
Eigen::Vector3d ApplyRelativeRotation(const Eigen::Vector3d& rotation1,
                                      const Eigen::Vector3d& relative_rotation);

// Computes the camera position given two oriented features seen by the camera
// and their corresponding 3d points. The features should have the effect of
// intrinsics removed and be oriented in the same coordinate system as the 3d
// points. The rays are constrained by:
//
//   depth * rotated_feature = X - c
//
// Such that rotated_feature = R^t * [u v 1]^t, with R being the known
// world-to-camera rotation.
bool PositionFromTwoRays(const Eigen::Vector2d& rotated_feature1,
                         const Eigen::Vector3d& point1,
                         const Eigen::Vector2d& rotated_feature2,
                         const Eigen::Vector3d& point2,
                         Eigen::Vector3d* position);

} // namespace tl
