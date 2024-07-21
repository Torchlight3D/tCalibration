#pragma once

#include <Eigen/SparseCore>

#include "rotationestimator.h"

namespace tl {

struct ViewPairInfo;

// Brief:
// Computes the orientation of views in a global frame given pairwise relative
// rotations between the views. This is done with a linear approximation to
// rotation averaging.
//
// Ref:
// "Robust Rotation and Translation Estimation in Multiview Geometry" by
// Martinec and Pajdla (CVPR 2007).
class LinearRotationEstimator : public RotationEstimator
{
public:
    using RotationEstimator::RotationEstimator;

    bool EstimateRotations(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        std::unordered_map<ViewId, Eigen::Vector3d>* rotations) override;

    // An alternative interface is to instead add relative rotation constraints
    // one by one with AddRelativeRotationConstraint, then call the
    // EstimateRotations interface below. This allows the caller to add multiple
    // constraints for the same view id pair, which may lead to more accurate
    // rotation estimates. Please see the following reference for an example of
    // how to obtain multiple constraints for pairs of views:
    //
    // Add the relative rotation matrix constraint that minimizes the
    // Frobenius norm of the matrices. That is, set up the linear system such
    // that
    // ||R_j - R_ij * R_i|| is minimized
    //
    // Ref:
    // "Parallel Structure from Motion from Local Increment to Global Averaging"
    // by Zhu et al (Arxiv 2017)
    void AddRelativeRotationConstraint(
        const ViewIdPair& viewPairId, const Eigen::Vector3d& relative_rotation);

    // Given the relative rotation constraints added with
    // AddRelativeRotationConstraint, this method returns the robust estimation
    // of global camera orientations. Like the method above, this requires an
    // initial estimate of the global orientations.
    bool EstimateRotations(
        std::unordered_map<ViewId, Eigen::Vector3d>* global_orientations);

private:
    // Lookup map to keep track of the global orientation estimates by view id.
    std::unordered_map<ViewId, int> _viewIds;

    // The sparse matrix is built up as new constraints are added.
    std::vector<Eigen::Triplet<double>> constraint_entries_;
};

} // namespace tl
