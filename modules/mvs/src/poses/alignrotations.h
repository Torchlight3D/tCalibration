#pragma once

#include <Eigen/Core>

namespace tl {

// Rotates the "rotation" set of orientations such that the orientations are
// most closely aligned in an L2 sense. That is, "rotation" is transformed such
// that R_rotation * R_gt_rotation^t is minimized.

// We solve a nonlinear least squares system to determine a rotation R that will
// align the rotation to the gt_rotation such that rotation * R = gt_rotation.
// This could potentially be set up as a linear system, however, that does not
// guarantee that R will remaind a valid rotation. Instead, we simply use a
// nonlinear system to ensure that R is a valid rotation.
void AlignRotations(const std::vector<Eigen::Vector3d>& gt_rotation,
                    std::vector<Eigen::Vector3d>* rotation);

} // namespace tl
