#pragma once

#include <Eigen/Core>

#include <tMath/Ransac/RansacCreator>

namespace tl {

struct Feature2D2D;
struct SacParameters;
struct SacSummary;

// Estimates a homography between two images given correspondences between
// images. Homographys are suited for mapping planar or rotation-only motion
// between two images and provide a one-to-one (i.e., pixel to pixel)
// mapping. The quality of the homography depends solely on the quality of the
// input data.
bool EstimateHomography(const SacParameters& ransacParams,
                        RansacType ransacType,
                        const std::vector<Feature2D2D>& corrs,
                        Eigen::Matrix3d* homography, SacSummary* ransacSummary);

// New interface
bool FindHomography(const std::vector<Feature2D2D>& corrs,
                    RansacType ransacType, const SacParameters& ransacParams,
                    Eigen::Matrix3d* homography, SacSummary* ransacSummary);

} // namespace tl
