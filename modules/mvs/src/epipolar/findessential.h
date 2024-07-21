#pragma once

#include <Eigen/Core>

#include <tMath/Ransac/RansacCreator>

namespace tl {

struct Feature2D2D;
struct SacParameters;
struct SacSummary;

struct RelativePose
{
    Eigen::Matrix3d essential_matrix;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
};

// Brief:
// Estimates the relative pose using the ransac.
//
// Inputs:
//     corrs: normalized (by intrinsics) 2D-2D correspondences
//     ransacType: RANSAC variant to use
//     ransacParams: RANSAC parameters
// Outputs:
//     pose: Relative pose
//     ransacSummary: RANSAC summary
// Return:
//     bool: if a pose could be succesfully estimated.

bool EstimateRelativePose(const SacParameters& ransacParams,
                          RansacType ransacType,
                          const std::vector<Feature2D2D>& corrs,
                          RelativePose* pose, SacSummary* ransacSummary);

// New interface
bool FindEssential(const std::vector<Feature2D2D>& corrs, RansacType ransacType,
                   const SacParameters& ransacParams, RelativePose* pose,
                   SacSummary* ransacSummary);

} // namespace tl
