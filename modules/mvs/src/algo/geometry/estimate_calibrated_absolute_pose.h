#pragma once

#include <vector>

#include <Eigen/Core>

#include <tMath/Types>
#include <tMvs/Types>

namespace tl {

struct CalibratedAbsolutePose
{
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
};

struct FeatureCorrespondence2D3D;
struct SacParameters;
struct SacSummary;

// Estimates the calibrated absolute pose using the ransac variant of choice
// (e.g. Ransac, Prosac, etc.). Correspondences must be normalized by the camera
// intrinsics. Returns true if a pose could be succesfully estimated, and false
// otherwise. The quality of the result depends on the quality of the input
// data.

// TODO: add other SAC algorithms
bool EstimateCalibratedAbsolutePose(
    const SacParameters& ransacParams, RansacType ransacType, PnPType pnpType,
    const std::vector<FeatureCorrespondence2D3D>& normalizedCorrespondences,
    CalibratedAbsolutePose* absolutePose, SacSummary* summary);

} // namespace tl
