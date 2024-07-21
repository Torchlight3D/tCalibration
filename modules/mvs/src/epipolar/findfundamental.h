#pragma once

#include <Eigen/Core>

#include <tMath/Ransac/RansacCreator>

namespace tl {

struct Feature2D2D;
struct SacParameters;
struct SacSummary;

// Relative pose information computed from two uncalibrated views. It is assumed
// that the first view has identity rotation and a position at the origin.
struct UncalibratedRelativePose
{
    Eigen::Matrix3d fmatrix;
    double focalLength1;
    double focalLength2;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
};

// Estimates the relative pose and focal lengths using the ransac variant of
// choice (e.g. Ransac, Prosac, etc.). Correspondences must be centered such
// that the principal point is (0, 0). Returns true if a pose could be
// succesfully estimated, and false otherwise. The quality of the result depends
// on the quality of the input data.
bool FindFundamental(const std::vector<Feature2D2D>& matches,
                     double minFocalLength, double maxFocalLength,
                     RansacType ransacType, const SacParameters& ransacParams,
                     UncalibratedRelativePose* res, SacSummary* ransacSummary);

} // namespace tl
