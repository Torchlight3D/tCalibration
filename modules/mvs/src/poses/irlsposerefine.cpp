#include "irlsposerefine.h"

#include <iomanip>
#include <iostream>

#include <glog/logging.h>

#include <tCore/ContainerUtils>
#include <tCore/Timer>
#include <tMath/Eigen/Rotation>
#include <tMath/Solvers/SparseCholeskyLLT>
#include <tMvs/ViewPairInfo>

#include "estimaterotationutils.h"

namespace tl {

using Eigen::ArrayXd;
using Eigen::Vector3d;

using SparseMatrixd = Eigen::SparseMatrix<double>;

IRLSRotationLocalRefiner::IRLSRotationLocalRefiner(int num_orientations,
                                                   int num_edges,
                                                   const Options& options)
    : options_(options)
{
    // The rotation change is one less than the number of global rotations
    // because we keep one rotation constant.
    tangent_space_step_.resize((num_orientations - 1) * 3);
    tangent_space_residual_.resize(num_edges * 3);
}

void IRLSRotationLocalRefiner::SetInitTangentSpaceStep(
    const Eigen::VectorXd& tangent_space_step)
{
    tangent_space_step_ = tangent_space_step;
}

void IRLSRotationLocalRefiner::SetViewIdToIndex(
    const std::unordered_map<ViewId, int>& view_id_to_index)
{
    view_id_to_index_ = view_id_to_index;
}

void IRLSRotationLocalRefiner::SetSparseMatrix(
    const Eigen::SparseMatrix<double>& sparse_matrix)
{
    sparse_matrix_ = sparse_matrix;
}

bool IRLSRotationLocalRefiner::solve(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& relative_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations)
{
    const int num_edges = relative_rotations.size();

    CHECK_NOTNULL(global_rotations);
    CHECK_GT(global_rotations->size(), 0);
    CHECK_GT(num_edges, 0);

    if (view_id_to_index_.empty()) {
        ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
    }

    if (sparse_matrix_.rows() == 0) {
        SetupLinearSystem(relative_rotations, (*global_rotations).size(),
                          view_id_to_index_, &sparse_matrix_);
    }

    // Set up the linear solver and analyze the sparsity pattern of the
    // system. Since the sparsity pattern will not change with each linear solve
    // this can help speed up the solution time.
    SparseCholeskyLLt linear_solver;
    linear_solver.analyzePattern(sparse_matrix_.transpose() * sparse_matrix_);
    if (linear_solver.info() != Eigen::Success) {
        LOG(ERROR) << "Cholesky decomposition failed.";
        return false;
    }

    LOG(INFO) << std::setw(12) << std::setfill(' ') << "Iter " << std::setw(16)
              << std::setfill(' ') << "SqError " << std::setw(16)
              << std::setfill(' ') << "Delta ";

    ComputeResiduals(relative_rotations, global_rotations);

    ArrayXd weights(num_edges * 3);
    SparseMatrixd at_weight;
    Timer timer;
    for (int i = 0; i < options_.max_num_irls_iterations; i++) {
        // Compute the Huber-like weights for each error term.
        const double& sigma = options_.irls_loss_parameter_sigma;
#pragma omp parallel for num_threads(options_.num_threads)
        for (int k = 0; k < num_edges; ++k) {
            double e_sq =
                tangent_space_residual_.segment<3>(3 * k).squaredNorm();
            double tmp = e_sq + sigma * sigma;
            double w = sigma / (tmp * tmp);
            weights.segment<3>(3 * k).setConstant(w);
        }

        // Update the factorization for the weighted values.
        at_weight = sparse_matrix_.transpose() * weights.matrix().asDiagonal();
        linear_solver.factorize(at_weight * sparse_matrix_);
        if (linear_solver.info() != Eigen::Success) {
            LOG(ERROR) << "Failed to factorize the least squares system.";
            return false;
        }

        // Solve the least squares problem.
        tangent_space_step_ =
            linear_solver.solve(at_weight * tangent_space_residual_);
        if (linear_solver.info() != Eigen::Success) {
            LOG(ERROR) << "Failed to solve the least squares system.";
            return false;
        }

        UpdateGlobalRotations(global_rotations);
        ComputeResiduals(relative_rotations, global_rotations);
        const double avg_step_size = ComputeAverageStepSize();

        LOG(INFO) << std::setw(12) << std::setfill(' ') << i << std::setw(16)
                  << std::setfill(' ') << tangent_space_residual_.squaredNorm()
                  << std::setw(16) << std::setfill(' ') << avg_step_size;

        if (avg_step_size < options_.irls_step_convergence_threshold) {
            LOG(INFO) << "IRLS Converged in " << i + 1 << " iterations.";
            break;
        }
    }

    LOG(INFO) << "Total time [IRLS]: " << timer.elapseInMs() << " ms.";
    return true;
}

void IRLSRotationLocalRefiner::UpdateGlobalRotations(
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations)
{
    for (auto& rotation : *global_rotations) {
        const int view_index =
            con::FindOrDie(view_id_to_index_, rotation.first) - 1;
        if (view_index == kConstantRotationIndex) {
            continue;
        }

        // Apply the rotation change to the global orientation.
        const Vector3d rotation_change =
            tangent_space_step_.segment<3>(3 * view_index);
        rotation.second = MultiplyRotations(rotation.second, rotation_change);
    }
}

void IRLSRotationLocalRefiner::ComputeResiduals(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& relative_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations)
{
    int rotation_error_index = 0;

    for (const auto& [viewIdPair, info] : relative_rotations) {
        const auto& relative_rotation_aa = info.rotation;
        const auto& rotation1 =
            con::FindOrDie(*global_rotations, viewIdPair.first);
        const auto& rotation2 =
            con::FindOrDie(*global_rotations, viewIdPair.second);

        // Compute the relative rotation error as:
        //   R_err = R2^t * R_12 * R1.
        tangent_space_residual_.segment<3>(3 * rotation_error_index) =
            MultiplyRotations(
                -rotation2, MultiplyRotations(relative_rotation_aa, rotation1));
        ++rotation_error_index;
    }
}

double IRLSRotationLocalRefiner::ComputeAverageStepSize() const
{
    // compute the average step size of the update in tangent_space_step_
    const int num_vertices = tangent_space_step_.size() / 3;

    double delta_V = 0;
    for (int k = 0; k < num_vertices; ++k) {
        delta_V += tangent_space_step_.segment<3>(3 * k).norm();
    }
    return delta_V / num_vertices;
}

} // namespace tl
