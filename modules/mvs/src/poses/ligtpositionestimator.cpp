#include "ligtpositionestimator.h"

#include <ceres/rotation.h>
#include <Eigen/SparseLU>
#include <glog/logging.h>
#include <Spectra/SymEigsShiftSolver.h>

#include <tCore/ContainerUtils>
#include <tCore/HashUtils>
#include <tMath/Solvers/SparseSymShiftSolveLLT>
#include <tMvs/Scene>

namespace tl {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using SparseMatrixd = Eigen::SparseMatrix<double>;
using Tripletd = Eigen::Triplet<double>;

namespace {

// We keep one of the positions as constant to remove the ambiguity of the
// origin of the linear system.
constexpr int kConstantPositionIndex = -1;

Matrix3d GetSkew(const Eigen::Vector3d& f)
{
    Matrix3d skew_mat;
    skew_mat << 0.0, -f(2), f(1), f(2), 0.0, -f(0), -f(1), f(0), 0.0;
    return skew_mat;
}

double GetThetaSq(const Eigen::Vector3d& feat_i, const Eigen::Vector3d& feat_j,
                  const Eigen::Matrix3d& Rij)
{
    return (GetSkew(feat_j) * Rij * feat_i).squaredNorm();
}

void AddTripletConstraintToSymmetricMatrix(
    const std::vector<Eigen::Matrix3d>& constraints,
    const std::vector<int>& viewIndices,
    std::unordered_map<std::pair<int, int>, double>* entries)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Skip any block entries that correspond to the lower triangular
            // portion of the matrix.
            // if (view_indices[i] > view_indices[j]) {
            //     continue;
            // }

            // Compute the A^t * B, etc. matrix.
            const Matrix3d symmetric_constraint =
                constraints[i].transpose() * constraints[j];

            // Add to the 3x3 block corresponding to (i, j)
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    const std::pair<int, int> row_col(viewIndices[i] + r,
                                                      viewIndices[j] + c);
                    (*entries)[row_col] += symmetric_constraint(r, c);
                }
            }
        }
    }
}

Eigen::Matrix3d AngleAxisToRotationMatrix(const Eigen::Vector3d& rvec)
{
    Matrix3d R;
    ceres::AngleAxisToRotationMatrix(rvec.data(), R.data());
    return R;
}

// Returns true if the vector R1 * (c2 - c1) is in the same direction as t_12.
bool VectorsAreSameDirection(const Eigen::Vector3d& position1,
                             const Eigen::Vector3d& position2,
                             const Eigen::Matrix3d& rotation1,
                             const Eigen::Vector3d& relative_position12)
{
    const Vector3d global_relative_position =
        (position2 - position1).normalized();
    const Vector3d rotated_relative_position =
        rotation1 * global_relative_position;

    return rotated_relative_position.dot(relative_position12) > 0;
}

// Returns the features as a unit-norm pixel ray after camera intrinsics
// (i.e. focal length an principal point) have been removed.
Feature GetNormalizedFeature(const View& view, TrackId trackId)
{
    const auto feature = *view.featureOf(trackId);
    const auto& camera = view.camera();
    const Vector3d ray = camera.pixelToNormalizedCoordinates(feature.pos);
    Feature normalized_Feature(ray.hnormalized());
    // todo normalized covariance?
    return normalized_Feature;
}

} // namespace

LiGTPositionEstimator::LiGTPositionEstimator(const Options& opts,
                                             const Scene& scene)
    : PositionEstimator(), _opts(opts), _scene(scene)
{
    CHECK_GT(opts.num_threads, 0);
}

LiGTPositionEstimator::~LiGTPositionEstimator() = default;

bool LiGTPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    CHECK_NOTNULL(positions)->clear();

    _viewIdToNumTriplets.clear();
    linear_system_index_.clear();
    _BCDs.clear();
    _trackIdToTriplets.clear();
    _viewPairs = &viewPairs;
    for (const auto& [viewId, rvec] : orientations) {
        _orientations[viewId] = AngleAxisToRotationMatrix(rvec);
    }

    VLOG(2)
        << "Extracting triplets from tracks and calculating BCDs for tracks.";

    FindTripletsForTracks();

    VLOG(2) << "Finished extracting triplets from tracks and calculating "
               "BCDs for tracks.";

    VLOG(2) << "Building the constraint matrix...";
    // Create the linear system based on triplet constraints.
    SparseMatrixd constraint_matrix;
    CreateLinearSystem(&constraint_matrix);

    // Solve for positions by examining the smallest eigenvalues. Since we have
    // set one position constant at the origin, we only need to solve for the
    // eigenvector corresponding to the smallest eigenvalue. This can be done
    // efficiently with inverse power iterations.
    VectorXd solution;
    if (orientations.size() > _opts.max_num_views_svd) {
        VLOG(2)
            << "Solving for positions from the sparse eigenvalue problem...";

        auto solver = std::make_shared<SparseCholeskyLLt>();
        SparseSymShiftSolveLLT<double> op{solver, constraint_matrix};
        Spectra::SymEigsShiftSolver eigs{op, 1, 6, 0.};
        eigs.init();
        eigs.compute(Spectra::SortRule::LargestMagn, 1000, 1e-4);
        solution = eigs.eigenvectors().col(0);
    }
    else {
        VLOG(2) << "Solving for positions from the eigenvalue problem using "
                   "BDCSVD...";

        MatrixXd constraint_matrix_dense(constraint_matrix);
        Eigen::BDCSVD<MatrixXd> svd{constraint_matrix_dense,
                                    Eigen::ComputeThinU | Eigen::ComputeThinV};
        solution = svd.matrixV().col(svd.matrixV().cols() - 1);
    }

    // Compute with power iterations.
    // Add the solutions to the output. Set the position with an index of -1 to
    // be at the origin.
    for (const auto& [viewId, index] : linear_system_index_) {
        if (index < 0) {
            (*positions)[viewId].setZero();
        }
        else {
            (*positions)[viewId] = solution.segment<3>(index * 3);
        }
    }

    FlipSignOfPositionsIfNecessary(positions);

    return true;
}

void LiGTPositionEstimator::FindTripletsForTracks()
{
    const auto trackIds = _scene.trackIds();

#pragma omp parallel for shared(num_triplets_for_view_, linear_system_index_, \
                                    triplets_for_tracks_, BCDs_)
    for (const auto& trackId : trackIds) {
        const auto observingViewIds = _scene.track(trackId)->viewIds();
        if (observingViewIds.size() < 3) {
            continue;
        }

        const auto [baseViewId1, baseViewId2] = GetBestBaseViews(trackId);
        // Iterate all other observations except the base views
        for (size_t v = 0; v < observingViewIds.size(); ++v) {
            const ViewId curViewId = *std::next(observingViewIds.begin(), v);
            if (curViewId == baseViewId1) {
                continue;
            }

            const auto BCD =
                calcBCDForTrack(baseViewId1, curViewId, baseViewId2, trackId);

            // const auto view1 = _scene.view(baseViewId1);
            // const auto view2 = _scene.view(curViewId);
            // const auto view3 = _scene.view(baseViewId2);
            // const auto& [B, C, D] = BCD;
            // const Vector3d zero = C * view1->camera().position() +
            //                       B * view2->camera().position() +
            //                       D * view3->camera().position();
            // LOG(INFO) << "Should be (close to) zero: " << zero;

#pragma omp critical
            {
                const ViewIdTriplet viewIdTriplet =
                    std::make_tuple(baseViewId1, curViewId, baseViewId2);

                const auto& [viewId1, viewId2, viewId3] = viewIdTriplet;

                _viewIdToNumTriplets[viewId1] += 1;
                _viewIdToNumTriplets[viewId2] += 1;
                _viewIdToNumTriplets[viewId3] += 1;

                // Determine the order of the views in the linear system. We
                // subtract 1 from the linear system index so that the first
                // position added to the system will be set constant (index of
                // -1 is intentionally not evaluated later).
                con::InsertIfNotPresent(&linear_system_index_, viewId1,
                                        linear_system_index_.size() - 1);
                con::InsertIfNotPresent(&linear_system_index_, viewId2,
                                        linear_system_index_.size() - 1);
                con::InsertIfNotPresent(&linear_system_index_, viewId3,
                                        linear_system_index_.size() - 1);

                _trackIdToTriplets[trackId].push_back(viewIdTriplet);
                _BCDs[trackId].push_back(BCD);
            }
        }
    }
}

std::pair<ViewId, ViewId> LiGTPositionEstimator::GetBestBaseViews(
    TrackId trackId) const
{
    const auto* track = _scene.track(trackId);
    const auto obserbingViewIds = track->viewIds();
    const std::vector<ViewId> viewIds{obserbingViewIds.cbegin(),
                                      obserbingViewIds.cend()};

    auto thetaMax{0.};
    std::pair<ViewId, ViewId> bestBaseViews;
    for (size_t i{0}; i < viewIds.size(); ++i) {
        const auto& viewId1 = viewIds[i];
        const auto* view1 = _scene.view(viewId1);
        const Vector3d feature1 =
            GetNormalizedFeature(*view1, trackId).pos.homogeneous();
        const Matrix3d R1 = _orientations.at(viewId1);

        for (size_t j = i + 1; j < viewIds.size(); ++j) {
            const auto& viewId2 = viewIds[j];
            const auto* view2 = _scene.view(viewId2);
            const Vector3d feature2 =
                GetNormalizedFeature(*view2, trackId).pos.homogeneous();
            const Matrix3d R2 = _orientations.at(viewId2);

            const Matrix3d R_12 = R2 * R1.transpose();
            const double theta = GetThetaSq(feature1, feature2, R_12);
            if (theta > thetaMax) {
                bestBaseViews = std::make_pair(viewId1, viewId2);
                thetaMax = theta;
            }
        }
    }

    return bestBaseViews;
}

LiGTPositionEstimator::BCD LiGTPositionEstimator::calcBCDForTrack(
    ViewId viewId1, ViewId viewId2, ViewId viewId3, TrackId trackId) const
{
    const auto view1 = _scene.view(viewId1);
    const auto view2 = _scene.view(viewId2);
    const auto view3 = _scene.view(viewId3);

    const Vector3d feat1 =
        GetNormalizedFeature(*view1, trackId).pos.homogeneous();
    const Vector3d feat2 =
        GetNormalizedFeature(*view2, trackId).pos.homogeneous();
    const Vector3d feat3 =
        GetNormalizedFeature(*view3, trackId).pos.homogeneous();

    const Matrix3d R1 = _orientations.at(viewId1);
    const Matrix3d R2 = _orientations.at(viewId2);
    const Matrix3d R3 = _orientations.at(viewId3);

    const Matrix3d R_31 = R1 * R3.transpose();
    const Matrix3d R_32 = R2 * R3.transpose();

    const Vector3d a32 =
        (GetSkew(R_32 * feat3) * feat2).transpose() * GetSkew(feat2);

    const Matrix3d skew_feat1 = GetSkew(feat1);
    // Eq.18
    const Matrix3d B = skew_feat1 * R_31 * feat3 * a32.transpose() * R2;

    const double theta = GetThetaSq(feat3, feat2, R_32);
    const Matrix3d C = theta * skew_feat1 * R1;

    const Matrix3d D = -(B + C);

    return {B, C, D};
}

void LiGTPositionEstimator::CreateLinearSystem(
    Eigen::SparseMatrix<double>* constraint_matrix)
{
    constexpr auto kEntryPerTriplet = Matrix3d::SizeAtCompileTime * 3;
    const auto viewCount = _viewIdToNumTriplets.size();

    // Retrieve entries from triplet. Entry: index -> value
    std::unordered_map<std::pair<int, int>, double> entries;
    entries.reserve(kEntryPerTriplet * viewCount);
    for (const auto& [trackId, triplets] : _trackIdToTriplets) {
        for (size_t i{0}; i < triplets.size(); ++i) {
            const auto& [viewId1, viewId2, viewId3] = triplets[i];
            const auto& [B, C, D] = _BCDs[trackId][i];

            // Weight each term by the inverse of the number of triplet that the
            // nodes participate in.
            // const double w =
            //     1. / std::sqrt(std::min({_viewIdToNumTriplets[viewId1],
            //                              _viewIdToNumTriplets[viewId2],
            //                              _viewIdToNumTriplets[viewId3]}));

            // Get the index of each camera in the sparse matrix.
            const std::vector viewIndices{
                3 * con::FindOrDie(linear_system_index_, viewId1),
                3 * con::FindOrDie(linear_system_index_, viewId2),
                3 * con::FindOrDie(linear_system_index_, viewId3)};

            // Eq.17. First pos (B) is the central camera
            const std::vector constraints{C, B, D};
            AddTripletConstraintToSymmetricMatrix(constraints, viewIndices,
                                                  &entries);
        }
    }

    // Set the sparse matrix from the container of the accumulated entries.
    std::vector<Tripletd> triplets;
    triplets.reserve(entries.size());
    for (const auto& [index, value] : entries) {
        // Skip this entry if the indices are invalid. This only occurs when we
        // encounter a constraint with the constant camera (which has a view
        // index of -1).
        if (index.first < 0 || index.second < 0) {
            continue;
        }

        triplets.emplace_back(index.first, index.second, value);
    }

    // We construct the constraint matrix A^t * A directly, which is an
    // N - 1 x N - 1 matrix where N is the number of cameras (and 3 entries per
    // camera, corresponding to the camera position entries).
    constraint_matrix->resize((viewCount - 1) * 3, (viewCount - 1) * 3);
    constraint_matrix->setFromTriplets(triplets.begin(), triplets.end());
    triplets.clear();
}

void LiGTPositionEstimator::FlipSignOfPositionsIfNecessary(
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    // Check the relative translation of views 1 and 2 in the triplet.
    // Score if they have same direction, give otherwise
    auto score{0};
    for (const auto& [viewPairId, viewPairInfo] : *_viewPairs) {
        const auto& viewId1 = viewPairId.first;
        const auto& viewId2 = viewPairId.second;

        const auto* position1 = con::FindOrNull(*positions, viewId1);
        const auto* position2 = con::FindOrNull(*positions, viewId2);
        if (!position1 || !position2) {
            continue;
        }

        if (VectorsAreSameDirection(
                *position1, *position2,
                con::FindOrDieNoPrint(_orientations, viewId1),
                viewPairInfo.position)) {
            score += 1;
        }
        else {
            score -= 1;
        }
    }

    // If score is below zero, meaning more than half of the view pairs have
    // inconsistent direction, then we should flip the sign of all estimated
    // positions.
    if (score < 0) {
        const int correctSign = (_viewPairs->size() + score) / 2;

        VLOG(2) << "Sign of the positions was incorrect: " << correctSign
                << " of " << _viewPairs->size()
                << " relative translations had the correct sign. "
                   "Flipping the sign of the camera positions.";

        for (auto& [_, position] : *positions) {
            position *= -1.;
        }
    }
}

} // namespace tl
