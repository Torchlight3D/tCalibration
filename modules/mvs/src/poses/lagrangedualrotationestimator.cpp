#include "lagrangedualrotationestimator.h"

#include <omp.h>
#include <algorithm>
#include <cmath>

#include <ceres/rotation.h>
#include <glog/logging.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/SymEigsSolver.h>

#include <tMath/SDP/RiemannianStaircaseSDPSolver>

#include "estimaterotationutils.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector3d;

using SparseMatrixd = Eigen::SparseMatrix<double>;
using Tripletd = Eigen::Triplet<double>;

LagrangeDualRotationEstimator::LagrangeDualRotationEstimator(
    const math::SDPSolver::Options& options)
    : options_(options)
{
    alpha_max_ = 0.;
}

void LagrangeDualRotationEstimator::SetViewIdToIndex(
    const std::unordered_map<ViewId, int>& view_id_to_index)
{
    view_id_to_index_ = view_id_to_index;
}

void LagrangeDualRotationEstimator::SetRAOption(
    const math::SDPSolver::Options& options)
{
    options_ = options;
}

const math::SDPSolver::Summary& LagrangeDualRotationEstimator::GetRASummary()
    const
{
    return summary_;
}

double LagrangeDualRotationEstimator::GetErrorBound() const
{
    return alpha_max_;
}

bool LagrangeDualRotationEstimator::EstimateRotations(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& view_pairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations)
{
    images_num_ = global_rotations->size();
    dim_ = 3;
    const size_t N = images_num_;

    CHECK(!view_pairs.empty());
    CHECK_GT(N, 0);

    R_ = SparseMatrixd(dim_ * N, dim_ * N);

    if (view_id_to_index_.empty()) {
        ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
    }

    // Set for R_
    std::unordered_map<size_t, std::vector<size_t>> adj_edges;
    FillinRelativeGraph(view_pairs, R_, adj_edges);

    auto solver = std::make_unique<math::RiemannianStaircase>(
        N, dim_, math::RiemannianStaircase::Options(), options_);

    solver->SetCovariance(-R_);
    solver->SetAdjacentEdges(adj_edges);
    summary_ = solver->Solve();
    Y_ = solver->GetSolution();

    RetrieveRotations(Y_, global_rotations);

    LOG(INFO) << "LagrangeDual converged in " << summary_.total_iterations_num
              << " iterations.";
    LOG(INFO) << "Total time [LagrangeDual]: " << summary_.TotalTime()
              << " ms.";

    return true;
}

void LagrangeDualRotationEstimator::ComputeErrorBound(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& view_pairs)
{
    const int N = images_num_;

    std::vector<Tripletd> a_triplets;
    // adjacency matrix
    SparseMatrixd A(N, N);
    // degree matrix (a diagonal matrix)
    SparseMatrixd D(N, N);
    std::vector<Tripletd> d_triplets;
    std::vector<double> degrees(N, 0);

    for (auto& view_pair : view_pairs) {
        ViewIdPair pair = view_pair.first;
        const int i = view_id_to_index_[pair.first];
        const int j = view_id_to_index_[pair.second];
        degrees[i]++;
        degrees[j]++;
    }

    double max_degree = {0.};
    for (auto& view_pair : view_pairs) {
        ViewIdPair pair = view_pair.first;
        const int i = view_id_to_index_[pair.first];
        const int j = view_id_to_index_[pair.second];

        a_triplets.push_back(Tripletd(i, j, 1.0));
        a_triplets.push_back(Tripletd(j, i, 1.0));

        d_triplets.push_back(Tripletd(i, i, degrees[i]));
        d_triplets.push_back(Tripletd(j, j, degrees[j]));
        max_degree = std::max({max_degree, degrees[i], degrees[j]});
    }

    A.setFromTriplets(a_triplets.begin(), a_triplets.end());
    A.makeCompressed();
    D.setFromTriplets(d_triplets.begin(), d_triplets.end());
    D.makeCompressed();

    // laplacian matrix
    SparseMatrixd L = D - A;

    // compute the bound of residual error
    // compute the bound of residual error
    Spectra::SparseSymMatProd<double> op(L);
    Spectra::SymEigsSolver<Spectra::SparseSymMatProd<double>> eigs(op, 2, 5);
    eigs.init();
    eigs.compute(Spectra::SortRule::SmallestAlge);

    double lambda2 = 0.0;
    if (eigs.info() == Spectra::CompInfo::Successful) {
        lambda2 = eigs.eigenvalues()[0];
    }
    else {
        LOG(INFO) << "Computing Eigenvalue fails";
    }

    // get the second smallest eigen value
    alpha_max_ =
        2 * std::asin(std::sqrt(0.25 + lambda2 / (2.0 * max_degree)) - 0.5);
}

void LagrangeDualRotationEstimator::RetrieveRotations(
    const Eigen::MatrixXd& Y,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations)
{
    for (const auto& [viewId, _] : *global_rotations) {
        const int i = view_id_to_index_[viewId];
        // After fixing Equ.(10)
        Matrix3d R = Y.block(0, 3 * i, 3, 3).transpose();
        if (R.determinant() < 0)
            R = -R;

        // CHECK_GE(R.determinant(), 0);
        // CHECK_NEAR(R.determinant(), 1, 1e-8);

        Vector3d angle_axis;
        ceres::RotationMatrixToAngleAxis(R.data(), angle_axis.data());
        // LOG(INFO) << angle_axis.norm();
        (*global_rotations)[viewId] = angle_axis;
    }
}

void LagrangeDualRotationEstimator::FillinRelativeGraph(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& view_pairs,
    Eigen::SparseMatrix<double>& R,
    std::unordered_map<size_t, std::vector<size_t>>& adj_edges)
{
    std::vector<Tripletd> triplets;
    for (auto it = view_pairs.begin(); it != view_pairs.end(); ++it) {
        // ViewId i = it->first.first, j = it->first.second;
        const int i = view_id_to_index_[it->first.first];
        const int j = view_id_to_index_[it->first.second];
        // CHECK_LT(i, j);
        Matrix3d R_ij;
        ceres::AngleAxisToRotationMatrix(it->second.rotation.data(),
                                         R_ij.data());

        // After fixing Eq.(9)
        // R.block(3 * i, 3 * j, 3, 3) = R_ij.transpose();
        // R.block(3 * j, 3 * i, 3, 3) = R_ij;
        for (int l = 0; l < 3; l++) {
            for (int r = 0; r < 3; r++) {
                triplets.push_back(
                    Tripletd(dim_ * i + l, dim_ * j + r, R_ij(r, l)));
                triplets.push_back(
                    Tripletd(dim_ * j + l, dim_ * i + r, R_ij(l, r)));
            }
        }

        adj_edges[i].push_back(j);
        adj_edges[j].push_back(i);
    }
    R.setFromTriplets(triplets.begin(), triplets.end());
    R.makeCompressed();
}

} // namespace tl
