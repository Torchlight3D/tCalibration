#include "robustrotationestimator.h"

#include <format>

#include <ceres/rotation.h>

#include <tCore/ContainerUtils>
#include <tMath/Eigen/Rotation>
#include <tMath/Solvers/L1Solver>

namespace tl {

using Eigen::ArrayXd;
using Eigen::Vector3d;

using SparseMatrixd = Eigen::SparseMatrix<double>;

namespace {
// We keep one of the rotations as constant to remove the ambiguity of the
// linear system.
constexpr int kConstantRotationIndex = -1;
} // namespace

RobustRotationEstimator::RobustRotationEstimator(const Options& options)
    : options_(options)
{
}

bool RobustRotationEstimator::EstimateRotations(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* g_orientations)
{
    for (const auto& [viewIdPair, info] : viewPairs) {
        AddRelativeRotationConstraint(viewIdPair, info.rotation);
    }

    return EstimateRotations(g_orientations);
}

void RobustRotationEstimator::AddRelativeRotationConstraint(
    const ViewIdPair& view_id_pair, const Eigen::Vector3d& relative_rotation)
{
    // Store the relative orientation constraint.
    relative_rotations_.emplace_back(view_id_pair, relative_rotation);
}

bool RobustRotationEstimator::EstimateRotations(
    std::unordered_map<ViewId, Eigen::Vector3d>* global_orientations)
{
    CHECK(!relative_rotations_.empty())
        << "Relative rotation constraints must be added to the robust rotation "
           "solver before estimating global rotations.";
    global_orientations_ = CHECK_NOTNULL(global_orientations);

    if (fixed_view_ids_.empty()) {
        // just set the first rotation fix
        fixed_view_ids_.insert(std::begin(*global_orientations)->first);
        nr_fixed_rotations_ = 1;
    }

    // Compute a mapping of view ids to indices in the linear system. One
    // rotation will have an index of -1 and will not be added to the linear
    // system. This will remove the gauge freedom (effectively holding one
    // camera as the identity rotation).
    int index = 0;
    int fix_index = -nr_fixed_rotations_;
    view_id_to_index_.reserve(global_orientations->size());
    for (const auto& [viewId, _] : *global_orientations) {
        if (fixed_view_ids_.find(viewId) == fixed_view_ids_.end()) {
            view_id_to_index_[viewId] = index;
            ++index;
        }
        else {
            view_id_to_index_[viewId] = fix_index;
            ++fix_index;
        }
    }

    SparseMatrixd sparse_mat;
    setupLinearSystem();

    if (!SolveL1Regression()) {
        LOG(ERROR) << "Could not solve the L1 regression step.";
        return false;
    }

    if (!SolveIRLS()) {
        LOG(ERROR) << "Could not solve the least squares error step.";
        return false;
    }

    return true;
}

void RobustRotationEstimator::setupLinearSystem()
{
    // The rotation change is one less than the number of global rotations
    // because we keep one rotation constant.
    tangent_space_step_.resize(
        (global_orientations_->size() - nr_fixed_rotations_) * 3);
    tangent_space_residual_.resize(relative_rotations_.size() * 3);
    sparse_matrix_.resize(
        relative_rotations_.size() * 3,
        (global_orientations_->size() - nr_fixed_rotations_) * 3);

    // For each relative rotation constraint, add an entry to the sparse
    // matrix. We use the first order approximation of angle axis such that:
    // R_ij = R_j - R_i. This makes the sparse matrix just a bunch of identity
    // matrices.
    int rotation_error_index = 0;
    std::vector<Eigen::Triplet<double>> triplets;
    for (const auto& [viewPairId, rotation] : relative_rotations_) {
        // see if this rotation should be fixed
        if (fixed_view_ids_.find(viewPairId.first) == fixed_view_ids_.end()) {
            const int view1_index =
                con::FindOrDie(view_id_to_index_, viewPairId.first);

            triplets.emplace_back(3 * rotation_error_index, 3 * view1_index,
                                  -1.0);
            triplets.emplace_back(3 * rotation_error_index + 1,
                                  3 * view1_index + 1, -1.0);
            triplets.emplace_back(3 * rotation_error_index + 2,
                                  3 * view1_index + 2, -1.0);
        }

        if (fixed_view_ids_.find(viewPairId.second) == fixed_view_ids_.end()) {
            const int view2_index =
                con::FindOrDie(view_id_to_index_, viewPairId.second);
            triplets.emplace_back(3 * rotation_error_index + 0,
                                  3 * view2_index + 0, 1.0);
            triplets.emplace_back(3 * rotation_error_index + 1,
                                  3 * view2_index + 1, 1.0);
            triplets.emplace_back(3 * rotation_error_index + 2,
                                  3 * view2_index + 2, 1.0);
        }

        ++rotation_error_index;
    }
    sparse_matrix_.setFromTriplets(triplets.begin(), triplets.end());
}

bool RobustRotationEstimator::SolveL1Regression()
{
    L1SolverOptions options;
    options.max_num_iterations = 5;
    L1Solver<SparseMatrixd> l1_solver(options, sparse_matrix_);

    tangent_space_step_.setZero();
    ComputeResiduals();
    for (int i = 0; i < options_.max_num_l1_iterations; i++) {
        tangent_space_step_ = l1_solver.Solve(tangent_space_residual_);
        UpdateGlobalRotations();
        ComputeResiduals();

        double avg_step_size = ComputeAverageStepSize();

        if (avg_step_size <= options_.l1_step_convergence_threshold) {
            break;
        }
        options.max_num_iterations *= 2;
        l1_solver.SetMaxIterations(options.max_num_iterations);
    }
    return true;
}

bool RobustRotationEstimator::SolveIRLS()
{
    const int num_edges = tangent_space_residual_.size() / 3;

    // Set up the linear solver and analyze the sparsity pattern of the
    // system. Since the sparsity pattern will not change with each linear solve
    // this can help speed up the solution time.
    SparseCholeskyLLt linear_solver;
    linear_solver.analyzePattern(sparse_matrix_.transpose() * sparse_matrix_);
    if (linear_solver.info() != Eigen::Success) {
        LOG(ERROR) << "Cholesky decomposition failed.";
        return false;
    }

    VLOG(2) << std::format("{:^20}{:^20}{:^20}", "Iteration", "SqError",
                           "Delta");

    ComputeResiduals();

    ArrayXd weights(num_edges * 3);
    SparseMatrixd at_weight;
    for (int i = 0; i < options_.max_num_irls_iterations; i++) {
        // Compute the Huber-like weights for each error term.
        const double& sigma = options_.irls_loss_parameter_sigma;
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

        // Solve the least squares problem..
        tangent_space_step_ =
            linear_solver.solve(at_weight * tangent_space_residual_);
        if (linear_solver.info() != Eigen::Success) {
            LOG(ERROR) << "Failed to solve the least squares system.";
            return false;
        }

        UpdateGlobalRotations();
        ComputeResiduals();
        const double avg_step_size = ComputeAverageStepSize();

        VLOG(2) << std::format("{:4d}{:4.4e}{:4.4e}", i,
                               tangent_space_residual_.squaredNorm(),
                               avg_step_size);

        if (avg_step_size < options_.irls_step_convergence_threshold) {
            VLOG(1) << "IRLS Converged in " << i + 1 << " iterations.";
            break;
        }
    }
    return true;
}

// Update the global orientations using the current value in the
// rotation_change.
void RobustRotationEstimator::UpdateGlobalRotations()
{
    for (auto& [viewId, orientation] : *global_orientations_) {
        if (fixed_view_ids_.find(viewId) != fixed_view_ids_.end()) {
            continue;
        }

        const int view_index = con::FindOrDie(view_id_to_index_, viewId);
        // Apply the rotation change to the global orientation.
        const Vector3d& rotation_change =
            tangent_space_step_.segment<3>(3 * view_index);
        orientation = MultiplyRotations(orientation, rotation_change);
    }
}

// Computes the relative rotation error based on the current global
// orientation estimates.
void RobustRotationEstimator::ComputeResiduals()
{
    int rotation_error_index = 0;
    for (const auto& [viewIdPair, rotation] : relative_rotations_) {
        const Vector3d& rotation1 =
            con::FindOrDie(*global_orientations_, viewIdPair.first);
        const Vector3d& rotation2 =
            con::FindOrDie(*global_orientations_, viewIdPair.second);

        // Compute the relative rotation error as:
        //   R_err = R2^t * R_12 * R1.
        tangent_space_residual_.segment<3>(3 * rotation_error_index) =
            MultiplyRotations(-rotation2,
                              MultiplyRotations(rotation, rotation1));
        ++rotation_error_index;
    }
}

double RobustRotationEstimator::ComputeAverageStepSize()
{
    // compute the average step size of the update in tangent_space_step_
    const int numVertices = tangent_space_step_.size() / 3;
    double delta_V = 0;
    for (int k = 0; k < numVertices; ++k) {
        delta_V += tangent_space_step_.segment<3>(3 * k).norm();
    }
    return delta_V / numVertices;
}

void RobustRotationEstimator::SetFixedGlobalRotations(
    const std::set<ViewId>& fixed_views)
{
    fixed_view_ids_ = fixed_views;
    nr_fixed_rotations_ = fixed_view_ids_.size();
}

} // namespace tl
