#include "linearpositionestimator.h"

#include <ceres/rotation.h>
#include <Eigen/SparseLU>
#include <glog/logging.h>
#include <Spectra/SymEigsShiftSolver.h>

#include <tCore/ContainerUtils>
#include <tCore/HashUtils>
#include <tCore/ThreadPool>
#include <tMath/Solvers/SparseSymShiftSolveLLT>
#include <tMvs/Scene>
#include <tMvs/ViewPairInfo>
#include <tMvs/Graph/TripletExtractor>
#include <tMvs/Epipolar/Triangulation>

namespace tl {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using SparseMatrixd = Eigen::SparseMatrix<double>;
using Tripletd = Eigen::Triplet<double>;

namespace {

// We keep one of the positions as constant to remove the ambiguity of the
// origin of the linear system.
constexpr int kConstantPositionIndex = -1;

std::vector<ViewIdTriplet> GetLargetConnectedTripletGraph(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs)
{
    constexpr int kLargestCCIndex = 0;

    // Get a list of all edges in the view graph.
    std::unordered_set<std::pair<ViewId, ViewId>> viewIdPairs;
    viewIdPairs.reserve(viewPairs.size());
    for (const auto& [viewPairId, _] : viewPairs) {
        viewIdPairs.insert(viewPairId.toPair());
    }

    // Extract connected triplets.
    TripletExtractor<ViewId> extractor;
    std::vector<std::vector<ViewIdTriplet>> triplets;
    CHECK(extractor.ExtractTriplets(viewIdPairs, &triplets));
    CHECK(!triplets.empty());
    return triplets[kLargestCCIndex];
}

// Adds the constraint from the triplet to the symmetric matrix. Our standard
// constraint matrix A is a 3M x 3N matrix with M triplet constraints and N
// cameras. We seek to construct A^t * A directly. For each triplet constraint
// in our matrix A (i.e. a 3-row block), we can compute the corresponding
// entries in A^t * A with the following summation:
//
//   A^t * A += Row(i)^t * Row(i)
//
// for each triplet constraint i.
void AddTripletConstraintToSymmetricMatrix(
    const std::array<Eigen::Matrix3d, 3>& constraints,
    const std::vector<int>& viewIndices,
    std::unordered_map<std::pair<int, int>, double>* entries)
{
    // Construct Row(i)^t * Row(i). If we denote the row as a block matrix:
    //
    //   Row(i) = [A | B | C]
    //
    // then we have:
    //
    //   Row(i)^t * Row(i) = [A | B | C]^t * [A | B | C]
    //                     = [ A^t * A  |  A^t * B  |  A^t * C]
    //                       [ B^t * A  |  B^t * B  |  B^t * C]
    //                       [ C^t * A  |  C^t * B  |  C^t * C]
    //
    // Since A^t * A is symmetric, we only store the upper triangular portion.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Skip any block entries that correspond to the lower triangular
            // portion of the matrix.
            if (viewIndices[i] > viewIndices[j]) {
                continue;
            }

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

// FIXME: Duplicated code
inline Matrix3d AngleAxisToRotationMatrix(const Eigen::Vector3d angle_axis)
{
    const double angle = angle_axis.norm();
    const AngleAxisd rotation_aa(angle, angle_axis / angle);
    return rotation_aa.toRotationMatrix();
}

// Returns true if the vector R1 * (c2 - c1) is in the same direction as t_12.
bool VectorsAreSameDirection(const Eigen::Vector3d& position1,
                             const Eigen::Vector3d& position2,
                             const Eigen::Vector3d& rotation1,
                             const Eigen::Vector3d& relative_position12)
{
    const Vector3d global_relative_position =
        (position2 - position1).normalized();
    Vector3d rotated_relative_position;
    ceres::AngleAxisRotatePoint(rotation1.data(),
                                global_relative_position.data(),
                                rotated_relative_position.data());
    return rotated_relative_position.dot(relative_position12) > 0;
}

// TODO: Duplicated
Feature GetNormalizedFeature(const View& view, TrackId trackId)
{
    const auto* feature = view.featureOf(trackId);
    const auto& camera = view.camera();
    Vector3d ray = camera.pixelToNormalizedCoordinates(feature->pos);
    Feature normalized_Feature(ray.hnormalized());
    // todo normalized covariance?
    return normalized_Feature;
}

} // namespace

LinearPositionEstimator::LinearPositionEstimator(const Options& options,
                                                 const Scene& reconstruction)
    : options_(options), reconstruction_(reconstruction)
{
    CHECK_GT(options.num_threads, 0);
}

bool LinearPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    CHECK_NOTNULL(positions)->clear();
    _viewPairs = &view_pairs;
    orientations_ = &orientations;

    // Extract triplets from the view pairs. As of now, we only consider the
    // largest connected triplet in the viewing graph.
    VLOG(2) << "Extracting triplets from the viewing graph.";
    triplets_ = GetLargetConnectedTripletGraph(view_pairs);

    VLOG(2) << "Determining baseline ratios within each triplet...";
    // Baselines where (x, y, z) corresponds to the baseline of the first,
    // second, and third view pair in the triplet.
    BS::thread_pool pool{static_cast<BS::concurrency_t>(options_.num_threads)};
    baselines_.resize(triplets_.size());
    for (size_t i = 0; i < triplets_.size(); i++) {
        AddTripletConstraint(triplets_[i]);
        pool.detach_task([this, i]() {
            ComputeBaselineRatioForTriplet(triplets_[i], &baselines_[i]);
        });
    }
    pool.wait();

    VLOG(2) << "Building the constraint matrix...";
    // Create the linear system based on triplet constraints.
    SparseMatrixd constraint_matrix;
    CreateLinearSystem(&constraint_matrix);

    // Solve for positions by examining the smallest eigenvalues. Since we have
    // set one position constant at the origin, we only need to solve for the
    // eigenvector corresponding to the smallest eigenvalue. This can be done
    // efficiently with inverse power iterations.
    VLOG(2) << "Solving for positions from the sparse eigenvalue problem...";
    auto solver = std::make_shared<SparseCholeskyLLt>();
    SparseSymShiftSolveLLT<double> op{solver, constraint_matrix};
    Spectra::SymEigsShiftSolver eigs{op, 1, 6, 0.0};
    eigs.init();
    eigs.compute(Spectra::SortRule::LargestMagn, 1000, 1e-6);

    // Compute with power iterations.
    const VectorXd solution = eigs.eigenvectors().col(0);

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

    // Flip the sign of the positions if necessary.
    FlipSignOfPositionsIfNecessary(positions);

    return true;
}

void LinearPositionEstimator::AddTripletConstraint(
    const ViewIdTriplet& viewTripletId)
{
    const auto& [viewId1, viewId2, viewId3] = viewTripletId;

    _viewIdToNumTriplets[viewId1] += 1;
    _viewIdToNumTriplets[viewId2] += 1;
    _viewIdToNumTriplets[viewId3] += 1;

    // Determine the order of the views in the linear system. We subtract 1 from
    // the linear system index so that the first position added to the system
    // will be set constant (index of -1 is intentionally not evaluated later).
    con::InsertIfNotPresent(&linear_system_index_, viewId1,
                            linear_system_index_.size() - 1);
    con::InsertIfNotPresent(&linear_system_index_, viewId2,
                            linear_system_index_.size() - 1);
    con::InsertIfNotPresent(&linear_system_index_, viewId3,
                            linear_system_index_.size() - 1);
}

void LinearPositionEstimator::ComputeBaselineRatioForTriplet(
    const ViewIdTriplet& viewIdTriplet, Eigen::Vector3d* baseline) const
{
    const auto& [viewId1, viewId2, viewId3] = viewIdTriplet;
    const View& view1 = *reconstruction_.view(viewId1);
    const View& view2 = *reconstruction_.view(viewId2);
    const View& view3 = *reconstruction_.view(viewId3);

    // Find common tracks.
    const std::vector viewIds{viewId1, viewId2, viewId3};
    const auto commonTrackIds = reconstruction_.trackIdsInViews(viewIds);

    // Normalize all features.
    std::vector<Feature> feature1, feature2, feature3;
    feature1.reserve(commonTrackIds.size());
    feature2.reserve(commonTrackIds.size());
    feature3.reserve(commonTrackIds.size());
    for (const auto& trackId : commonTrackIds) {
        feature1.push_back(GetNormalizedFeature(view1, trackId).pos);
        feature2.push_back(GetNormalizedFeature(view2, trackId).pos);
        feature3.push_back(GetNormalizedFeature(view3, trackId).pos);
    }

    // Get the baseline ratios.
    ViewTripletInfo viewTriplet;
    viewTriplet.ids[0] = viewId1;
    viewTriplet.ids[1] = viewId2;
    viewTriplet.ids[2] = viewId3;
    viewTriplet.info12 =
        con::FindOrDieNoPrint(*_viewPairs, ViewIdPair{viewId1, viewId2, false});
    viewTriplet.info13 =
        con::FindOrDieNoPrint(*_viewPairs, ViewIdPair{viewId1, viewId3, false});
    viewTriplet.info23 =
        con::FindOrDieNoPrint(*_viewPairs, ViewIdPair{viewId2, viewId3, false});

    if (!ComputeTripletBaselineRatios(viewTriplet, feature1, feature2, feature3,
                                      baseline)) {
        baseline->setZero();
    }
}

void LinearPositionEstimator::CreateLinearSystem(
    Eigen::SparseMatrix<double>* constraint_matrix)
{
    const int num_views = _viewIdToNumTriplets.size();

    std::unordered_map<std::pair<int, int>, double> entries;
    entries.reserve(27 * _viewIdToNumTriplets.size());
    for (int i = 0; i < triplets_.size(); i++) {
        const auto& [viewId1, viewId2, viewId3] = triplets_[i];
        AddTripletConstraintToSparseMatrix(viewId1, viewId2, viewId3,
                                           baselines_[i], &entries);
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
    constraint_matrix->resize((num_views - 1) * 3, (num_views - 1) * 3);
    constraint_matrix->setFromTriplets(triplets.begin(), triplets.end());
}

void LinearPositionEstimator::ComputeRotatedRelativeTranslationRotations(
    ViewId viewId1, ViewId viewId2, ViewId viewId3, Eigen::Matrix3d* R_123,
    Eigen::Matrix3d* r201, Eigen::Matrix3d* r120) const
{
    // Relative camera positions.
    const Vector3d& rvec1 = con::FindOrDieNoPrint(*orientations_, viewId1);
    const Vector3d& rvec2 = con::FindOrDieNoPrint(*orientations_, viewId2);
    const Matrix3d R1 = AngleAxisToRotationMatrix(rvec1);
    const Matrix3d R2 = AngleAxisToRotationMatrix(rvec2);

    const Vector3d t_12 =
        -R1.transpose() *
        con::FindOrDieNoPrint(*_viewPairs, ViewIdPair{viewId1, viewId2, false})
            .position;
    const Vector3d t_13 =
        -R1.transpose() *
        con::FindOrDieNoPrint(*_viewPairs, ViewIdPair{viewId1, viewId3, false})
            .position;
    const Vector3d t_23 =
        -R2.transpose() *
        con::FindOrDieNoPrint(*_viewPairs, ViewIdPair{viewId2, viewId3, false})
            .position;

    // Rotations between the translation vectors.
    *R_123 = Quaterniond::FromTwoVectors(t_23, -t_12).toRotationMatrix();
    *r201 = Quaterniond::FromTwoVectors(t_12, t_13).toRotationMatrix();
    *r120 = Quaterniond::FromTwoVectors(-t_13, -t_23).toRotationMatrix();
}

void LinearPositionEstimator::AddTripletConstraintToSparseMatrix(
    ViewId viewId1, ViewId viewId2, ViewId viewId3,
    const Eigen::Vector3d& baselines,
    std::unordered_map<std::pair<int, int>, double>* entries)
{
    // Weight each term by the inverse of the number of triplet that the nodes
    // participate in.
    const auto w = 1. / std::sqrt(std::min({_viewIdToNumTriplets[viewId1],
                                            _viewIdToNumTriplets[viewId2],
                                            _viewIdToNumTriplets[viewId3]}));

    // Get the index of each camera in the sparse matrix.
    const std::vector viewIndices{
        3 * con::FindOrDie(linear_system_index_, viewId1),
        3 * con::FindOrDie(linear_system_index_, viewId2),
        3 * con::FindOrDie(linear_system_index_, viewId3)};

    // Compute the rotations between relative translations.
    Matrix3d R_123, R_312, R_231;
    ComputeRotatedRelativeTranslationRotations(viewId1, viewId2, viewId3,
                                               &R_123, &R_312, &R_231);

    // Baselines ratios.
    const double s_123 = baselines[0] / baselines[2];
    const double s_312 = baselines[1] / baselines[0];
    const double s_231 = baselines[2] / baselines[1];

    const auto I33 = Matrix3d::Identity();

    // Assume t01 is perfect and solve for c2.
    {
        std::array<Matrix3d, 3> constraints;
        constraints[0] = (-s_312 * R_312 + R_123.transpose() / s_123 + I33) * w;
        constraints[1] = (s_312 * R_312 - R_123.transpose() / s_123 + I33) * w;
        constraints[2] = -2. * w * I33;
        AddTripletConstraintToSymmetricMatrix(constraints, viewIndices,
                                              entries);
    }

    // Assume t02 is perfect and solve for c1.
    {
        std::array<Matrix3d, 3> constraints;
        constraints[0] = (-R_312.transpose() / s_312 + s_231 * R_231 + I33) * w;
        constraints[1] = -2. * w * I33;
        constraints[2] = (R_312.transpose() / s_312 - s_231 * R_231 + I33) * w;
        AddTripletConstraintToSymmetricMatrix(constraints, viewIndices,
                                              entries);
    }

    // Assume t12 is perfect and solve for c0.
    {
        std::array<Matrix3d, 3> constraints;
        constraints[0] = -2. * w * I33;
        constraints[1] = (-s_123 * R_123 + R_231.transpose() / s_231 + I33) * w;
        constraints[2] = (s_123 * R_123 - R_231.transpose() / s_231 + I33) * w;
        AddTripletConstraintToSymmetricMatrix(constraints, viewIndices,
                                              entries);
    }
}

void LinearPositionEstimator::FlipSignOfPositionsIfNecessary(
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
                con::FindOrDieNoPrint(*orientations_, viewId1),
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

namespace {

// Triangulate the point and return the depth of the point relative to each
// view. Returns true if the depth was recovered successfully and false if the
// point could not be triangulated.
bool GetTriangulatedPointDepths(const ViewPairInfo& info,
                                const Eigen::Vector3d& feature1,
                                const Eigen::Vector3d& feature2, double* depth1,
                                double* depth2)
{
    constexpr double kMinTriangulationAngle = 2.0;

    const std::vector<Vector3d> origins{Vector3d::Zero(), info.position};

    Matrix3d R2;
    ceres::AngleAxisToRotationMatrix(info.rotation.data(),
                                     ceres::ColumnMajorAdapter3x3(R2.data()));

    const std::vector<Vector3d> directions{feature1, R2.transpose() * feature2};

    // Make sure the rays have are viewed from a sufficient angle, otherwise the
    // depth computation is unstable.
    if (!SufficientTriangulationAngle(directions, kMinTriangulationAngle)) {
        return false;
    }

    Vector4d point;
    if (!TriangulateMidpoint(origins, directions, &point)) {
        return false;
    }

    // Compute depths.
    const Vector3d point3d = point.hnormalized();
    *depth1 = point3d.norm();
    *depth2 = (point3d - info.position).norm();
    return true;
}

} // namespace

bool ComputeTripletBaselineRatios(const ViewTripletInfo& triplet,
                                  const std::vector<Feature>& feature1,
                                  const std::vector<Feature>& feature2,
                                  const std::vector<Feature>& feature3,
                                  Eigen::Vector3d* baseline)
{
    CHECK_NOTNULL(baseline)->setZero();
    CHECK_EQ(feature1.size(), feature2.size())
        << "The feature containers must be the same size when computing the "
           "triplet baseline ratios.";
    CHECK_EQ(feature1.size(), feature3.size())
        << "The feature containers must be the same size when computing the "
           "triplet baseline ratios.";

    Vector4d point12, point13, point23;
    double depth1_12, depth2_12, depth1_13, depth3_13, depth2_23, depth3_23;

    std::vector<double> baseline2, baseline3;
    baseline2.reserve(feature2.size());
    baseline3.reserve(feature3.size());
    for (int i = 0; i < feature1.size(); i++) {
        const Vector3d normalized_feature1 =
            feature1[i].pos.homogeneous().normalized();
        const Vector3d normalized_feature2 =
            feature2[i].pos.homogeneous().normalized();
        const Vector3d normalized_feature3 =
            feature3[i].pos.homogeneous().normalized();

        if (!GetTriangulatedPointDepths(triplet.info12, normalized_feature1,
                                        normalized_feature2, &depth1_12,
                                        &depth2_12)) {
            continue;
        }

        // Compute triangulation from views 1, 3.
        if (!GetTriangulatedPointDepths(triplet.info13, normalized_feature1,
                                        normalized_feature3, &depth1_13,
                                        &depth3_13)) {
            continue;
        }

        // Compute triangulation from views 2, 3.
        if (!GetTriangulatedPointDepths(triplet.info23, normalized_feature2,
                                        normalized_feature3, &depth2_23,
                                        &depth3_23)) {
            continue;
        }

        baseline2.emplace_back(depth1_12 / depth1_13);
        baseline3.emplace_back(depth2_12 / depth2_23);
    }

    if (baseline2.empty()) {
        VLOG(2)
            << "Could not compute the triplet baseline ratios. An inusfficient "
               "number of well-constrained 3D points were observed.";
        return false;
    }

    // Take the median as the baseline ratios.
    const auto mid = baseline2.size() / 2;
    std::nth_element(baseline2.begin(), baseline2.begin() + mid,
                     baseline2.end());
    std::nth_element(baseline3.begin(), baseline3.begin() + mid,
                     baseline3.end());
    *baseline = Vector3d(1., baseline2[mid], baseline3[mid]);
    return true;
}

} // namespace tl
