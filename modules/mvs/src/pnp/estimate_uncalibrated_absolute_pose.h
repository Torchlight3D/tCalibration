﻿#pragma once

#include <Eigen/Core>

#include <tMath/Types>

namespace tl {

struct Feature2D3D;
struct SacParameters;
struct SacSummary;

struct UncalibratedAbsolutePose
{
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    double focal_length;
};

// Estimates the uncalibrated absolute pose using the ransac variant of choice
// (e.g. Ransac, Prosac, etc.). Correspondences must be normalized by the camera
// intrinsics. Returns true if a pose could be succesfully estimated, and false
// otherwise. The quality of the result depends on the quality of the input
// data.
bool EstimateUncalibratedAbsolutePose(
    const SacParameters& ransac_params, RansacType ransac_type,
    const std::vector<Feature2D3D>& normalized_correspondences,
    UncalibratedAbsolutePose* absolute_pose, SacSummary* ransac_summary);

} // namespace tl
