#pragma once

// #define EIGEN_USE_MKL_ALL

#include <tMath/SDP/SDPSolver>
#include "rotationestimator.h"

namespace tl {

// Brief:
// Given the pairwise relative rotations, retrive the globally optimal value
// of absolute orientations.
//
// Ref:
// "Rotation Averaging with strong duality." by Erikson. et.al. (CVPR 2018 &&
// PAMI 2019)
class LagrangeDualRotationEstimator : public RotationEstimator
{
public:
    // FIXME: Now default use RiemanniumStaircaseSDPSolver
    explicit LagrangeDualRotationEstimator(
        const math::SDPSolver::Options& option = {});

    void SetViewIdToIndex(const std::unordered_map<ViewId, int>& viewIdToIndex);

    void SetRAOption(const math::SDPSolver::Options& option);

    const math::SDPSolver::Summary& GetRASummary() const;

    bool EstimateRotations(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        std::unordered_map<ViewId, Eigen::Vector3d>* rotations) override;

    // Compute the upper bound of angular error alpha_max_
    // If for all |alpha_{ij}| < alpha_max_, the strong duality hold.
    void ComputeErrorBound(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs);
    double GetErrorBound() const;

private:
    // Retrieve optimal solutions from matrix Y_
    void RetrieveRotations(
        const Eigen::MatrixXd& Y,
        std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations);

    void FillinRelativeGraph(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        Eigen::SparseMatrix<double>& R,
        std::unordered_map<size_t, std::vector<size_t>>& adj_edges);

private:
    math::SDPSolver::Options options_;
    math::SDPSolver::Summary summary_;

    // number of images/frames
    size_t images_num_;

    size_t dim_;

    // the compact matrix representation in Equ.(9) of Eriksson's paper
    Eigen::SparseMatrix<double> R_;

    // the optimized variable of (DD) problem
    Eigen::MatrixXd Y_;

    // upper bound for strong duality hold
    double alpha_max_;

    // this hash table is used for non-continuous index, such as
    // unordered internet datasets that composed of many unconnected components
    std::unordered_map<ViewId, int> view_id_to_index_;
};

} // namespace tl
