#pragma once

#include <unordered_map>

#include <tMvs/ViewPairInfo>

namespace tl {

// Global position estimation methods.
//   Nonlinear: This method minimizes the nonlinear pairwise translation
//   constraint to solve for positions.
//
//   Linear: This linear method computes camera positions by minimizing an error
//   for image triplets. Essentially, it tries to enforce a loop/triangle
//   constraint for triplets.
//
//   LeastUnsqauredDeviation (LUD): This robust method uses the least unsquared
//   deviation instead of least squares. It is essentially an L1 solver.
//
//   LiGT:
//
// NOTE: The recommended method is LUD.
enum class GlobalPositionEstimatorType
{
    Nonlinear = 0,
    Linear,
    LUD,
    LiGT
};

// Brief:
// The abstract class defining the interface for global position estimation
// algorithms, which take the global (absolute) orientation estimated for each
// camera and pairwise translation directions between pairs of cameras as input.
// Additional information such as track/correspondences can also be passed in as
// needed, but those will be specific to the subclass implementation.
class PositionEstimator
{
public:
    PositionEstimator() = default;
    virtual ~PositionEstimator() = default;

    // Input the view pairs containing relative poses between matched
    // geometrically verified views, as well as the global (absolute)
    // orientations of the camera that were previously estimated.
    virtual bool EstimatePositions(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientation,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions) = 0;

private:
    // DISALLOW_COPY_AND_ASSIGN(PositionEstimator);
};

// Function style interface
bool EstimateGlobalPosition(
    GlobalPositionEstimatorType type,
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientation,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions);

} // namespace tl
