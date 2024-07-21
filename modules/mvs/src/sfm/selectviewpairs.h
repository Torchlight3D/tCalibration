#pragma once

#include <memory>
#include <unordered_map>

#include <Eigen/Core>

#include <tMvs/Types>

namespace tl {

class RandomNumberGenerator;
class ViewGraph;

namespace FilterViewPairs {

struct ByRelativeTranslationOptions
{
    // Random number generator used to sample from normal distributions to
    // compute the axis of projection for 1dsfm filtering.
    std::shared_ptr<RandomNumberGenerator> rng;

    // Filtering the translations is embarassingly parallel (each iteration can
    // be run independently) so we can use a thread pool to speed up
    // computation.
    int num_threads = 1;

    // The projection will be performed for the given number of iterations (we
    // recommend > 40 iterations).
    int num_iterations = 48;

    // The parameter translation_projection_tolerance determines which
    // translations are considered "bad" after analyzing their projections over
    // many iterations (it corresponds to tau in the paper).
    double translation_projection_tolerance = 0.08;
};

// Brief:
// Filters view pairs based on the relative translation estimations. This
// algorithm determines translations directions which are likely to be outliers
// by projecting translations to a 1-dimensional subproblem. The relative
// translations are repeatedly projected onto a (semi) random vector and are
// ordered to find a consistent embedding. Translation projections which are
// inconsistent with the ordering are likely to be outliers. This process is
// repeated over many iterations to determine translation directions likely to
// be outliers.
//
// Ref:
// "Robust Global Translations with 1DSfM" by Kyle Wilson and Noah Snavely (ECCV
// 2014)
void ByRelativeTranslations(
    const ByRelativeTranslationOptions& options,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    ViewGraph* viewGraph);

// Brief:
// Filter view pairs based on the estimated orientations. If the relative
// rotation obtained from the two view match differs from the relative rotation
// calculated from estimated orientations by more than the threshold, then the
// view pair is considered as outlier and is removed.
//
// Concretely, keep R_{i,j} if:
//           ||R_{i,j} - R_j * R_i^t|| < threshold,
// where || A - B || is the angular distance between A and B.
//
// NOTE: This function will also remove the view pairs that contain a view that
// does not have an entry in orientations.
void ByOrientations(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    double maxRelativeRotationDiff, ViewGraph* viewGraph);

} // namespace FilterViewPairs

// Brief:
// Extracts the maximally parallel rigid component of the subgraph based on
// known camera orientations and relative translation measurements.
//
// Given a set of relative translations, a rigid component is a subgraph that is
// well-posed such that global node positions may be recovered. For relative
// translations, this basically means that a triangle constraint exists on the
// node. Any nodes that are free to move (with scale or translation) with
// respect to the rigid component are not part of the rigid component. The goal
// of this method is to extract the largest rigid component so that we may
// obtain a well-posed graph for global position estimation.
//
// Ref:
// [1] "Identifying Maximal Rigid Components in Bearing-Based Localization" by
// Kennedy et al (IROS 2012)
// [2] "Robust Camera Location Estimation by Convex Programming" by Ozyesil and
// Singer (CVPR 2015)
void ExtractMaximallyParallelRigidSubgraph(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    ViewGraph* viewGraph);

} // namespace tl
