#pragma once

#include <unordered_map>

#include <Eigen/SparseCore>

#include <tCore/Math>
#include <tMvs/Types>

namespace tl {

struct ViewPairInfo;

// Brief:
// Refine global position by iteratively reweighted least squares (IRLS)
class IRLSRotationLocalRefiner
{
public:
    struct Options
    {
        int num_threads = 8;

        // The number of iterative reweighted least squares iterations to
        // perform.
        int max_num_irls_iterations = 10;

        // Average step size threshold to termininate the IRLS minimization
        double irls_step_convergence_threshold = 1e-3;

        // This is the point where the Huber-like cost function switches from L1
        // to L2.
        double irls_loss_parameter_sigma = math::degToRad(5.);
    };

    IRLSRotationLocalRefiner(int num_orientations, int num_edges,
                             const Options& options);

    void SetInitTangentSpaceStep(const Eigen::VectorXd& tangent_space_step);

    void SetViewIdToIndex(
        const std::unordered_map<ViewId, int>& view_id_to_index);

    void SetSparseMatrix(const Eigen::SparseMatrix<double>& sparse_matrix);

    bool solve(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& relative_rotations,
        std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations);

    // We keep one of the rotations as constant to remove the ambiguity of the
    // linear system.
    inline static constexpr int kConstantRotationIndex = -1;

private:
    // Update the global orientations using the current value in the
    // rotation_change.
    void UpdateGlobalRotations(
        std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations);

    // Computes the relative rotation error based on the current global
    // orientation estimates.
    void ComputeResiduals(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& relative_rotations,
        std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations);

    // Computes the average size of the most recent step of the algorithm.
    // The is the average over all non-fixed global_rotations_ of their
    // rotation magnitudes.
    double ComputeAverageStepSize() const;

private:
    const Options options_;

    // Map of ViewIds to the corresponding positions of the view's orientation
    // in the linear system.
    std::unordered_map<ViewId, int> view_id_to_index_;

    // The sparse matrix used to maintain the linear system. This is matrix A in
    // Ax = b.
    Eigen::SparseMatrix<double> sparse_matrix_;

    // x in the linear system Ax = b.
    Eigen::VectorXd tangent_space_step_;

    // b in the linear system Ax = b.
    Eigen::VectorXd tangent_space_residual_;
};

} // namespace tl
