#include "selectviewpairs.h"

#include <memory>
#include <mutex>
#include <unordered_map>

#include <ceres/rotation.h>
#include <Eigen/Geometry>

#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Rotation>
#include <tMath/Eigen/Utils>
#include <tMvs/ViewGraph>

#include "threadpool.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

// Helper struct to maintain the graph for the translation projection problem.
struct MFASNode
{
    std::unordered_map<ViewId, double> incoming_nodes;
    std::unordered_map<ViewId, double> outgoing_nodes;
    double incoming_weight = 0;
    double outgoing_weight = 0;
};

// Find the next view to add to the order. We attempt to choose a source (i.e.,
// a node with no incoming edges) or choose a node based on a heuristic such
// that it has the most source-like properties.
ViewId FindNextViewInOrder(
    const std::unordered_map<ViewId, MFASNode>& degrees_for_view)
{
    auto bestViewId = kInvalidViewId;
    auto bestScore = 0.;
    for (const auto& [viewId, node] : degrees_for_view) {
        // If the view is a source view, return it.
        if (node.incoming_nodes.empty()) {
            return viewId;
        }

        // Otherwise, keep track of the max score seen so far.
        const double score =
            (node.outgoing_weight + 1.) / (node.incoming_weight + 1.);
        if (score > bestScore) {
            bestViewId = viewId;
            bestScore = score;
        }
    }

    return bestViewId;
}

// Based on the 1D translation projections, compute an ordering of the
// translations.
std::unordered_map<ViewId, int> OrderTranslationsFromProjections(
    const std::unordered_map<ViewIdPair, double>&
        translation_direction_projections)
{
    // Compute the degrees of all vertices as the sum of weights coming in or
    // out.
    std::unordered_map<ViewId, MFASNode> degrees_for_view;
    for (const auto& [viewPairId, weight] : translation_direction_projections) {
        const ViewIdPair viewIdPair =
            (weight > 0.)
                ? viewPairId
                : ViewIdPair{viewPairId.second, viewPairId.first, false};

        // Update the MFAS entry.
        const double w = std::abs(weight);
        degrees_for_view[viewIdPair.second].incoming_weight += w;
        degrees_for_view[viewIdPair.first].outgoing_weight += w;
        degrees_for_view[viewIdPair.second].incoming_nodes.emplace(
            viewIdPair.first, w);
        degrees_for_view[viewIdPair.first].outgoing_nodes.emplace(
            viewIdPair.second, w);
    }

    // Compute the ordering.
    const int num_views = degrees_for_view.size();
    std::unordered_map<ViewId, int> translation_ordering;
    for (int i = 0; i < num_views; i++) {
        // Find the next view to add.
        const ViewId nextViewInOrder = FindNextViewInOrder(degrees_for_view);
        translation_ordering[nextViewInOrder] = i;

        // Update the MFAS graph and remove the next view from the
        // degrees_for_view.
        const auto& next_view_info =
            con::FindOrDie(degrees_for_view, nextViewInOrder);
        for (const auto& [viewId, weight] : next_view_info.incoming_nodes) {
            degrees_for_view[viewId].outgoing_weight -= weight;
            degrees_for_view[viewId].outgoing_nodes.erase(nextViewInOrder);
        }
        for (const auto& [viewId, weight] : next_view_info.outgoing_nodes) {
            degrees_for_view[viewId].incoming_weight -= weight;
            degrees_for_view[viewId].incoming_nodes.erase(nextViewInOrder);
        }
        degrees_for_view.erase(nextViewInOrder);
    }

    return translation_ordering;
}

// Projects all the of the translation onto the given axis.
std::unordered_map<ViewIdPair, double> ProjectTranslationsOntoAxis(
    const Eigen::Vector3d& axis,
    const std::unordered_map<ViewIdPair, Eigen::Vector3d>&
        relative_translations)
{
    std::unordered_map<ViewIdPair, double> projection_weights;
    projection_weights.reserve(relative_translations.size());
    for (const auto& [viewPairId, translation] : relative_translations) {
        const double projection_weight = translation.dot(axis);
        projection_weights.emplace(viewPairId, projection_weight);
    }
    return projection_weights;
}

void TranslationFilteringIteration(
    const std::unordered_map<ViewIdPair, Eigen::Vector3d>&
        relative_translations,
    const Eigen::Vector3d& direction_mean,
    const Eigen::Vector3d& direction_variance,
    const std::shared_ptr<RandomNumberGenerator>& rng, std::mutex* mutex,
    std::unordered_map<ViewIdPair, double>* edgeWeights)
{
    // Create the random number generator within each thread. If the random
    // number generator is not supplied then create a new one within each
    // thread.
    std::shared_ptr<RandomNumberGenerator> local_rng;
    if (!rng.get()) {
        local_rng = std::make_shared<RandomNumberGenerator>();
    }
    else {
        local_rng = rng;
    }

    // Get a random vector to project all relative translations on to.
    const Vector3d random_axis =
        Vector3d{local_rng->randNorm(direction_mean[0], direction_variance[0]),
                 local_rng->randNorm(direction_mean[1], direction_variance[1]),
                 local_rng->randNorm(direction_mean[2], direction_variance[2])}
            .normalized();

    // Project all vectors.
    const auto translation_direction_projections =
        ProjectTranslationsOntoAxis(random_axis, relative_translations);

    // Compute ordering.
    const auto translation_ordering =
        OrderTranslationsFromProjections(translation_direction_projections);

    // Compute bad edge weights.
    for (auto& [viewPairId, weight] : *edgeWeights) {
        const auto& [viewId1, viewId2] = viewPairId;

        const int ordering_diff =
            con::FindOrDie(translation_ordering, viewId2) -
            con::FindOrDie(translation_ordering, viewId1);
        const double& projection_weight_of_edge = con::FindOrDieNoPrint(
            translation_direction_projections, viewPairId);

        VLOG(3) << "Edge (" << viewId1 << ", " << viewId2
                << ") has ordering diff of " << ordering_diff
                << " and a projection of " << projection_weight_of_edge
                << " from "
                << con::FindOrDieNoPrint(relative_translations, viewPairId)
                       .transpose();

        // If the ordering is inconsistent, add the absolute value of the bad
        // weight to the aggregate bad weight.
        if ((ordering_diff < 0 && projection_weight_of_edge > 0) ||
            (ordering_diff > 0 && projection_weight_of_edge < 0)) {
            std::lock_guard<std::mutex> lock(*mutex);
            weight += std::abs(projection_weight_of_edge);
        }
    }
}

} // namespace

namespace {

Eigen::MatrixXd makeAngleMeasurementMatrix(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    const ViewGraph& viewGraph,
    const std::unordered_map<ViewId, int>& viewIdsToIndex)
{
    const auto& viewPairs = viewGraph.viewPairs();

    // Set up the matrix such that t_{i,j} x (c_j - c_i) = 0.
    MatrixXd mat(3 * viewGraph.numEdges(), 3 * orientations.size());
    mat.setZero();

    int i = 0;
    for (const auto& [id, info] : viewPairs) {
        const auto& [viewId1, viewId2] = id;

        // Get t_{i,j} and rotate it such that it is oriented in the global
        // reference frame.
        Matrix3d R_1;
        ceres::AngleAxisToRotationMatrix(
            con::FindOrDie(orientations, viewId1).data(),
            ceres::ColumnMajorAdapter3x3(R_1.data()));
        const Vector3d rotated_translation = R_1.transpose() * info.position;
        const Matrix3d cross_product_mat = math::Skew(rotated_translation);

        // Find the column locations of the two views.
        const int view1_col = 3 * con::FindOrDie(viewIdsToIndex, viewId1);
        const int view2_col = 3 * con::FindOrDie(viewIdsToIndex, viewId2);

        mat.block<3, 3>(3 * i, view1_col) = -cross_product_mat;
        mat.block<3, 3>(3 * i, view2_col) = cross_product_mat;
        ++i;
    }

    return mat;
}

// Find the maximal rigid component containing fixed_node. This is done by
// examining which nodes are parallel when removing node fixed_node from the
// null space. The nodes are only parallel if they are part of the maximal rigid
// component with fixed_node.
std::unordered_set<int> FindMaximalParallelRigidComponent(
    const Eigen::MatrixXd& nullspace, int fixed_node)
{
    // Computes the cosine distance in each dimension x, y, and z and returns
    // the maximum cosine distance.
    //
    // cos distance = 1.0 - a.dot(b) / (norm(a) * norm(b))
    auto calcCosineDistance = [](const Eigen::Matrix3Xd& mat1,
                                 const Eigen::Matrix3Xd& mat2) -> double {
        Vector3d dist;
        for (int i = 0; i < 3; i++) {
            dist(i) = 1. - std::abs(mat1.row(i).dot(mat2.row(i)));
        }
        return dist.maxCoeff();
    };

    constexpr double kMaxCosDistance = 1e-5;
    constexpr double kMaxNorm = 1e-10;

    const int num_nodes = nullspace.rows() / 3;

    std::unordered_set<int> largest_cc;

    largest_cc.insert(fixed_node);

    const MatrixXd fixed_null_space_component =
        nullspace.block(3 * fixed_node, 0, 3, nullspace.cols());

    // Remove the fixed node from the rest of the null space.
    MatrixXd modified_null_space =
        nullspace - fixed_null_space_component.replicate(num_nodes, 1);

    // Normalize all rows to be unit-norm. If the rows have a very small norm
    // then they are parallel to the fixed_node. and should be set to zero.
    const VectorXd norms = modified_null_space.rowwise().norm();

    modified_null_space.rowwise().normalize();

    // Find the pairs to match. Add all indices that are close to 0-vectors, as
    // they are clearly part of the rigid component.
    std::vector<int> indices_to_match;
    for (int i = 0; i < num_nodes; i++) {
        if (i == fixed_node) {
            continue;
        }

        // Skip this index if it is nearly a 0-vector because this means it is
        // clearly part of the rigid component.
        if (norms(3 * i) < kMaxNorm && norms(3 * i + 1) < kMaxNorm &&
            norms(3 * i + 2) < kMaxNorm) {
            largest_cc.insert(i);
            continue;
        }

        indices_to_match.emplace_back(i);
    }

    // Each node has three dimensions (x, y, z). We only compare parallel-ness
    // between similar dimensions. If all x, y, z dimensions are parallel then
    // the two nodes will be parallel.
    for (size_t i{0}; i < indices_to_match.size(); ++i) {
        // Test all other nodes (that have not been tested) to determine if they
        // are parallel to this node.
        const MatrixXd& block1 = modified_null_space.block(
            3 * indices_to_match[i], 0, 3, nullspace.cols());

        for (auto j = i + 1; j < indices_to_match.size(); ++j) {
            const MatrixXd& block2 = modified_null_space.block(
                3 * indices_to_match[j], 0, 3, nullspace.cols());

            const double cos_distance = calcCosineDistance(block1, block2);
            if (cos_distance < kMaxCosDistance) {
                largest_cc.insert(indices_to_match[i]);
                largest_cc.insert(indices_to_match[j]);
            }
        }
    }

    return largest_cc;
}

} // namespace

void ExtractMaximallyParallelRigidSubgraph(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    ViewGraph* viewGraph)
{
    std::unordered_map<ViewId, int> viewIdsToIndex;
    viewIdsToIndex.reserve(orientations.size());
    for (const auto& [viewId, _] : orientations) {
        if (!viewGraph->hasView(viewId)) {
            continue;
        }

        const int index = viewIdsToIndex.size();
        con::InsertIfNotPresent(&viewIdsToIndex, viewId, index);
    }

    // Form the global angle measurements matrix from:
    //    t_{i,j} x (c_j - c_i) = 0.
    MatrixXd angle_measurements =
        makeAngleMeasurementMatrix(orientations, *viewGraph, viewIdsToIndex);

    // Extract the null space of the angle measurements matrix.
    Eigen::FullPivLU<MatrixXd> lu(angle_measurements.transpose() *
                                  angle_measurements);
    const MatrixXd nullspace = lu.kernel();

    // For each node in the graph (i.e. each camera), set the null space
    // component to be zero such that the camera position would be fixed at the
    // origin. If two nodes i and j are in the same rigid component, then their
    // null spaces will be parallel because the camera positions may only change
    // by a scale. We find all components that are parallel to find the rigid
    // components. The largest of such component is the maximally parallel rigid
    // component of the graph.
    std::unordered_set<int> cc_max;
    for (int i = 0; i < orientations.size(); i++) {
        auto cc_tmp = FindMaximalParallelRigidComponent(nullspace, i);
        if (cc_tmp.size() > cc_max.size()) {
            std::swap(cc_tmp, cc_max);
        }
    }

    // Only keep the nodes in the largest maximally parallel rigid component.
    for (const auto& [viewId, _] : orientations) {
        const int index = con::FindOrDie(viewIdsToIndex, viewId);
        // If the view is not in the maximal rigid component then remove it from
        // the view graph.
        if (!cc_max.contains(index) && viewGraph->hasView(viewId)) {
            CHECK(viewGraph->removeView(viewId))
                << "Could not remove view id " << viewId
                << " from the view graph because it does not exist.";
        }
    }
}

namespace FilterViewPairs {

namespace internal {

// Rotate the translation direction based on the known orientation such that the
// translation is in the global reference frame.
std::unordered_map<ViewIdPair, Eigen::Vector3d>
rotateRelativeTranslationsToGlobalFrame(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations)
{
    std::unordered_map<ViewIdPair, Vector3d> g_translations;
    g_translations.reserve(viewPairs.size());
    for (const auto& [viewPairId, viewPairInfo] : viewPairs) {
        const Vector3d g_rotation =
            -1. * con::FindOrDie(orientations, viewPairId.first);
        Vector3d g_translation;
        ceres::AngleAxisRotatePoint(g_rotation.data(),
                                    viewPairInfo.position.data(),
                                    g_translation.data());
        g_translations.emplace(viewPairId, g_translation);
    }
    return g_translations;
}

void calcTranslationMeanVariance(
    const std::unordered_map<ViewIdPair, Eigen::Vector3d>& translations,
    Eigen::Vector3d* mean, Eigen::Vector3d* var)
{
    std::vector<Vector3d> tvecs;
    tvecs.reserve(translations.size());
    std::transform(translations.cbegin(), translations.cend(),
                   std::back_inserter(tvecs),
                   [](const auto& pair) { return pair.second; });

    Eigen::Map<Matrix3Xd> mat(tvecs[0].data(), 3, tvecs.size());
    *mean = mat.rowwise().mean();
    *var = (mat.colwise() - *mean).rowwise().squaredNorm() /
           (translations.size() - 1);
}

} // namespace internal

void ByRelativeTranslations(
    const ByRelativeTranslationOptions& opts,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    ViewGraph* viewGraph)
{
    const auto& viewPairs = viewGraph->edgesAndInfo();

    std::unordered_map<ViewIdPair, double> edgeWeights;
    edgeWeights.reserve(viewPairs.size());
    for (const auto& [viewPairId, _] : viewPairs) {
        edgeWeights.emplace(viewPairId, 0.);
    }

    // Transform relative translations to global frame with estimated global
    // orientations.
    const auto g_translations =
        internal::rotateRelativeTranslationsToGlobalFrame(viewPairs,
                                                          orientations);

    Vector3d translation_mean, translation_var;
    internal::calcTranslationMeanVariance(g_translations, &translation_mean,
                                          &translation_var);

    auto pool = std::make_unique<ThreadPool>(opts.num_threads);
    std::mutex mutex;
    for (int i = 0; i < opts.num_iterations; i++) {
        pool->Add(TranslationFilteringIteration, g_translations,
                  translation_mean, translation_var, opts.rng, &mutex,
                  &edgeWeights);
    }
    pool.reset(nullptr);

    // Remove all the bad edges.
    const double max_aggregated_projection_tolerance =
        opts.translation_projection_tolerance * opts.num_iterations;

    int num_view_pairs_removed = 0;
    for (const auto& [viewPairId, weight] : edgeWeights) {
        VLOG(3) << "View pair (" << viewPairId.first << ", "
                << viewPairId.second << ") projection: " << weight;

        if (weight > max_aggregated_projection_tolerance) {
            viewGraph->removeEdge(viewPairId.first, viewPairId.second);
            ++num_view_pairs_removed;
        }
    }

    VLOG(1) << "Removed " << num_view_pairs_removed
            << " view pairs by relative translation filtering.";
}

void ByOrientations(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    double maxAngleDiff, ViewGraph* viewGraph)
{
    CHECK_NOTNULL(viewGraph);
    CHECK_GE(maxAngleDiff, 0.);

    auto checkAngleDiff = [](const Eigen::Vector3d& orientation1,
                             const Eigen::Vector3d& orientation2,
                             const Eigen::Vector3d& relative_orientation,
                             double sq_maxAngleDiff) -> bool {
        const Vector3d composed_relative_rotation =
            MultiplyRotations(orientation2, -orientation1);
        const Vector3d loop_rotation = MultiplyRotations(
            -relative_orientation, composed_relative_rotation);
        const auto sq_angleDiff = loop_rotation.squaredNorm();
        return sq_angleDiff <= sq_maxAngleDiff;
    };

    const double maxAngleDiffInRad = math::degToRad(maxAngleDiff);
    const double sq_maxAngleDiffInRad = maxAngleDiffInRad * maxAngleDiffInRad;

    const auto& viewPairs = viewGraph->edgesAndInfo();

    std::unordered_set<ViewIdPair> viewPairIdsToRemove;
    for (const auto& [id, info] : viewPairs) {
        const auto& [viewId1, viewId2] = id;

        const auto* orientation1 = con::FindOrNull(orientations, viewId1);
        const auto* orientation2 = con::FindOrNull(orientations, viewId2);

        // If the view pair contains a view that does not have an orientation
        // then remove it.
        if (!orientation1 || !orientation2) {
            LOG(WARNING) << "View pair (" << viewId1 << ", " << viewId2
                         << ") contains a view that does not exist. "
                            "Removing the view pair.";
            viewPairIdsToRemove.insert(id);
            continue;
        }

        if (!checkAngleDiff(*orientation1, *orientation2, info.rotation,
                            sq_maxAngleDiffInRad)) {
            viewPairIdsToRemove.insert(id);
        }
    }

    for (const auto& viewPairId : viewPairIdsToRemove) {
        viewGraph->removeEdge(viewPairId.first, viewPairId.second);
    }

    VLOG(1) << "Removed " << viewPairIdsToRemove.size()
            << " view pairs by rotation filtering.";
}

} // namespace FilterViewPairs

} // namespace tl
