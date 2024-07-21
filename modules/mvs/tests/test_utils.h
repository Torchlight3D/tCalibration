#pragma once

#include <Eigen/Core>

#include <tMvs/Types>

namespace tl {

class RandomNumberGenerator;

void AddNoiseToVector2(double factor, Eigen::Vector2d* vec);

Eigen::Matrix3d RandomRotation(double angleInDeg);

// Aligns rotations to the ground truth rotations via a similarity
// transformation.
void AlignOrientations(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* rotations);

// Aligns positions to the ground truth positions via a similarity
// transformation.
void AlignPositions(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_positions,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions);

} // namespace tl
