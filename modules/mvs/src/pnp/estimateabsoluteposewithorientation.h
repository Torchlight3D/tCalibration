#pragma once

#include <Eigen/Core>

#include <tMath/Types>

namespace tl {

struct Feature2D3D;
struct SacParameters;
struct SacSummary;

// Estimates the absolute pose with known orientation. It is assumed that the 2D
// features in the 2D-3D correspondences are normalized by the camera
// intrinsics. Returns true if the position could be successfully estimated and
// false otherwise. The quality of the result depends on the quality of the
// input data.
bool EstimateAbsolutePoseWithKnownOrientation(
    const SacParameters& ransac_params, RansacType ransac_type,
    const Eigen::Vector3d& camera_orientation,
    const std::vector<Feature2D3D>& normalized_correspondences,
    Eigen::Vector3d* camera_position, SacSummary* ransac_summary);

namespace EstimateAbsolutePose {

bool WithKnownOrientation(const std::vector<Feature2D3D>& correspondences,
                          const Eigen::Vector3d& orientation,
                          RansacType ransacType,
                          const SacParameters& ransacParams,
                          Eigen::Vector3d* position, SacSummary* ransacSummary);
}

} // namespace tl
