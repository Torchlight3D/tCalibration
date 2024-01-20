#pragma once

#include <vector>
#include <Eigen/Core>

#include <tMath/RansacCreator>

namespace tl {

struct RadialDistUncalibratedAbsolutePose
{
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double focal_length;
    double radial_distortion;
};

// use this to sort out solutions that do not make sense
struct RadialDistUncalibratedAbsolutePoseMetaData
{
    double max_focal_length = 10000.;
    double min_focal_length = 200.;
    double min_radial_distortion = -1e-9;
    double max_radial_distortion = -1e-5;
};

struct FeatureCorrespondence2D3D;
struct RansacParameters;
struct RansacSummary;

// Estimates the uncalibrated absolute pose using the ransac variant of choice
// (e.g. Ransac, Prosac, etc.). Apart from the focal length, a radial distortion
// is estimated that can be used with the division undistortion camera model
// The quality of the result depends on the quality of the input
// data. The feature correspondences should be normalized such that
// the principal point is at (0, 0).
bool EstimateRadialDistUncalibratedAbsolutePose(
    const SacParameters& params, RansacType type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences,
    const RadialDistUncalibratedAbsolutePoseMetaData& meta_data,
    RadialDistUncalibratedAbsolutePose* absolute_pose,
    SacSummary* ransac_summary);

} // namespace tl
