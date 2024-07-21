#include "hybridrotationestimator.h"

#include <omp.h>

#include <ceres/rotation.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <glog/logging.h>

#include <tCore/ContainerUtils>

#include "estimaterotationutils.h"
#include "lagrangedualrotationestimator.h"

namespace tl {

using Eigen::Vector3d;
using Eigen::VectorXd;

using SparseMatrixd = Eigen::SparseMatrix<double>;

HybridRotationEstimator::HybridRotationEstimator(const Options& options)
    : options_(options),
      ld_rotation_estimator_(
          new LagrangeDualRotationEstimator(options.sdp_solver_options)),
      irls_rotation_refiner_(nullptr)
{
}

bool HybridRotationEstimator::EstimateRotations(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& view_pairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations)
{
    images_num_ = global_rotations->size();
    dim_ = 3;
    const int N = images_num_;

    CHECK_NOTNULL(global_rotations);
    CHECK_GT(N, 0);
    CHECK_EQ(N, (*global_rotations).size());
    CHECK(!view_pairs.empty());

    irls_rotation_refiner_.reset(new IRLSRotationLocalRefiner(
        N, view_pairs.size(), options_.irls_options));

    ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
    ld_rotation_estimator_->SetViewIdToIndex(view_id_to_index_);

    SparseMatrixd sparse_matrix;
    SetupLinearSystem(view_pairs, (*global_rotations).size(), view_id_to_index_,
                      &sparse_matrix);
    irls_rotation_refiner_->SetViewIdToIndex(view_id_to_index_);
    irls_rotation_refiner_->SetSparseMatrix(sparse_matrix);

    // Estimate global rotations that resides within the cone of
    // convergence for IRLS.
    LOG(INFO) << "Estimating Rotations Using LagrangeDual";
    ld_rotation_estimator_->EstimateRotations(view_pairs, global_rotations);

    // Refine the globally optimal result by IRLS.
    VectorXd tangent_space_step;
    GlobalRotationsToTangentSpace(*global_rotations, &tangent_space_step);
    irls_rotation_refiner_->SetInitTangentSpaceStep(tangent_space_step);

    LOG(INFO) << "Refining Global Rotations";
    irls_rotation_refiner_->solve(view_pairs, global_rotations);

    return true;
}

void HybridRotationEstimator::GlobalRotationsToTangentSpace(
    const std::unordered_map<ViewId, Eigen::Vector3d>& global_rotations,
    Eigen::VectorXd* tangent_space_step)
{
    (*tangent_space_step).resize((global_rotations.size() - 1) * 3);

    for (const auto& [viewId, rotation] : global_rotations) {
        const int view_index = con::FindOrDie(view_id_to_index_, viewId) - 1;
        if (view_index == IRLSRotationLocalRefiner::kConstantRotationIndex) {
            continue;
        }

        (*tangent_space_step).segment<3>(3 * view_index) = rotation;
    }
}

} // namespace tl
