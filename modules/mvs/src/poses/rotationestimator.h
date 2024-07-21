#pragma once

#include <unordered_map>

#include <tMvs/ViewPairInfo>

namespace tl {

// Global rotation estimation methods:
//   Robust:
//
//   Nonlinear:
//
//   Linear:
//
//   LagrangeDuality:
//
//   Hybrid:
//
// NOTE: The recommended method is Robust.
// This method is scalable, extremely accurate, and very efficient.
enum class GlobalRotationEstimatorType
{
    Robust = 0,
    Nonlinear = 1,
    Linear = 2,
    LagrangeDuality = 3,
    Hybrid = 4
};

// Brief:
// The abstract class defining the interface for global rotation estimation
// algorithms, which take the relative pairwise rotations as input, and estimate
// for the global orientation of each view.
class RotationEstimator
{
public:
    RotationEstimator() = default;
    virtual ~RotationEstimator() = default;

    // Input the view pairs containing relative rotations between matched
    // geometrically verified views and outputs a rotation estimate for each
    // view.
    virtual bool EstimateRotations(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        std::unordered_map<ViewId, Eigen::Vector3d>* rotations) = 0;

private:
    // DISALLOW_COPY_AND_ASSIGN(RotationEstimator);
};

// Function style interface
bool EstimateGlobalRotations(
    GlobalRotationEstimatorType type,
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* rotations);

} // namespace tl
