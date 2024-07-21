#pragma once

#include <Eigen/SparseCore>

#include "positionestimator.h"

#include "../desc/feature.h"

namespace tl {

class Scene;
class View;
struct ViewPairInfo;

// Brief:
// Estimates the camera position of views given global orientations and view
// triplets. The constraints formed by each triplet are used to create a sparse
// linear system to solve for the positions.
//
// Ref:
// "A Global Linear Method for Camera Pose Registration" by Jiang et al
// (ICCV2013)
class LinearPositionEstimator final : public PositionEstimator
{
public:
    struct Options
    {
        int num_threads = 1;

        // Maximum number of inverse power iterations to perform while
        // extracting the eigenvector corresponding to the smallest eigenvalue.
        int max_power_iterations = 1000;

        // The threshold at which to the iterative eigensolver method is
        // considered to be converged.
        double eigensolver_threshold = 1e-8;
    };

    LinearPositionEstimator(const Options& options, const Scene& scene);

    // Estimate the positions given view pairs and global orientation estimates.
    bool EstimatePositions(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientation,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions) override;

private:
    // Computes the relative baselines between three views in a triplet. The
    // baseline is estimated from the depths of triangulated 3D points. The
    // relative positions of the triplets are then scaled to account for the
    // baseline ratios.
    void ComputeBaselineRatioForTriplet(const ViewIdTriplet& triplet,
                                        Eigen::Vector3d* baseline) const;

    // Store the triplet.
    // An alternative interface is to instead add triplets one by one to linear
    // estimator. This allows for adding redundant observations of triplets,
    // which may be useful if there are multiple estimates of the data.
    void AddTripletConstraint(const ViewIdTriplet& triplet);

    // Sets up the linear system with the constraints that each triplet adds.
    void CreateLinearSystem(Eigen::SparseMatrix<double>* constraint_matrix);

    // Adds a triplet constraint to the linear system. The weight of the
    // constraint (w), the global orientations, baseline (ratios), and view
    // triplet information are needed to form the constraint.
    void AddTripletConstraintToSparseMatrix(
        ViewId viewId1, ViewId viewId2, ViewId viewId3,
        const Eigen::Vector3d& baseline,
        std::unordered_map<std::pair<int, int>, double>* entries);

    // A helper method to compute the relative rotations between translation
    // directions.
    void ComputeRotatedRelativeTranslationRotations(
        ViewId viewId1, ViewId viewId2, ViewId viewId3, Eigen::Matrix3d* R_123,
        Eigen::Matrix3d* R_312, Eigen::Matrix3d* R_231) const;

    // FIXME: Duplicated code
    void FlipSignOfPositionsIfNecessary(
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

private:
    const Options options_;
    const Scene& reconstruction_;
    const std::unordered_map<ViewIdPair, ViewPairInfo>* _viewPairs;
    const std::unordered_map<ViewId, Eigen::Vector3d>* orientations_;

    std::vector<ViewIdTriplet> triplets_;
    std::vector<Eigen::Vector3d> baselines_;

    std::unordered_map<ViewId, int> _viewIdToNumTriplets;
    std::unordered_map<ViewId, int> linear_system_index_;

    // DISALLOW_COPY_AND_ASSIGN(LinearPositionEstimator);
};

// The baselines of the relative poses between views are estimated by
// triangulating features common to all 3 views. Based on the depth of the
// triangulated features, the baseline between views is recovered. The order of
// the features must be aligned such that feature1[i] corresponds to feature2[i]
// and feature3[i].
//
// The baselines returned in the 3-vector correspond to the baseline between
// views 1 and 2, between views 1 and 3, and between views 2 and 3 in that
// order.
//
// NOTE: The features must be normalized by the camera intrinsics (i.e.,
// principal point and focal length must be removed).
bool ComputeTripletBaselineRatios(const ViewTripletInfo& triplet,
                                  const std::vector<Feature>& feature1,
                                  const std::vector<Feature>& feature2,
                                  const std::vector<Feature>& feature3,
                                  Eigen::Vector3d* baseline);

} // namespace tl
