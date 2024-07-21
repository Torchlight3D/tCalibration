#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>

#include "positionestimator.h"

namespace tl {

class RandomNumberGenerator;
class Scene;
class View;
struct ViewPairInfo;

// Computes the error between a translation direction and the direction formed
// from two positions such that (c_j - c_i) - scalar * t_ij is minimized.
struct PairwiseTranslationError
{
    const Eigen::Vector3d translation_direction_;
    const double weight_;
    const double scale_estimate_;

    PairwiseTranslationError(const Eigen::Vector3d& translation_direction,
                             double weight, double scale_estimate = -1.0)
        : translation_direction_(translation_direction),
          weight_(weight),
          scale_estimate_(scale_estimate)
    {
        CHECK_GT(weight_, 0);
    }

    // The error is given by the position error described above.
    template <typename T>
    bool operator()(const T* position1, const T* position2, T* residuals) const
    {
        using Vec3 = Eigen::Vector3<T>;

        const Eigen::Map<const Vec3> pos1(position1);
        const Eigen::Map<const Vec3> pos2(position2);

        const Vec3 t = pos2 - pos1;

        if (scale_estimate_ > 0.) {
            // If we have a scale estimate, then we want to estimate the
            // positions directlry
            Eigen::Map<Vec3>{residuals} =
                weight_ *
                (t - scale_estimate_ * translation_direction_.cast<T>());
            return true;
        }

        T norm = t.norm();

        // If the norm is very small then the positions are very close
        // together. In this case, avoid dividing by a tiny number which
        // will cause the weight of the residual term to potentially
        // skyrocket.
        const T kNormTolerance = T(1e-12);
        if (T(norm) < kNormTolerance) {
            norm = T(1);
        }

        Eigen::Map<Vec3>{residuals} =
            weight_ * (t / norm - translation_direction_.cast<T>());
        return true;
    }

    static ceres::CostFunction* create(
        const Eigen::Vector3d& translation_direction, double weight,
        double scale_estimate);
};

// Brief:
// Estimates the camera position of views given pairwise relative poses and the
// absolute orientations of cameras. Positions are estimated using a nonlinear
// solver with a robust cost function.
//
// Ref:
// "Robust Global Translations with 1DSfM" by Wilson and Snavely (ECCV 2014)
class NonlinearPositionEstimator : public PositionEstimator
{
public:
    struct Options
    {
        // The random number generator used to generate random numbers through
        // the reconstruction estimation process. If this is a nullptr then the
        // random generator will be initialized based on the current time.
        std::shared_ptr<RandomNumberGenerator> rng;

        // Options for Ceres nonlinear solver.
        int num_threads = 1;
        int max_num_iterations = 400;
        double robust_loss_width = 0.1;

        // Minimum number of 3D points to camera correspondences for each
        // camera. These points can help constrain the problem and add
        // robustness to collinear configurations, but are not necessary to
        // compute the position.
        int min_num_points_per_view = 0;

        // The total weight of all point to camera correspondences compared to
        // camera to camera correspondences.
        double point_to_camera_weight = 0.5;
    };

    NonlinearPositionEstimator(const Options& options, const Scene& scene);

    bool EstimatePositions(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientation,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions) override;

    bool EstimateRemainingPositionsInRecon(
        const std::set<ViewId>& fixed_views,
        const std::unordered_set<ViewId>& views_in_subrecon,
        const std::unordered_map<ViewIdPair, ViewPairInfo>&
            view_pairs_sub_recon,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

private:
    // Initialize all cameras to be random.
    void InitializeRandomPositions(
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

    // Creates camera to camera constraints from relative translations.
    void AddCameraToCameraConstraints(
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

    // Creates point to camera constraints.
    void AddPointToCameraConstraints(
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

    // Determines which tracks should be used for point to camera constraints. A
    // greedy approach is used so that the fewest number of tracks are chosen
    // such that all cameras have at least k point to camera constraints.
    int FindTracksForProblem(
        const std::unordered_map<ViewId, Eigen::Vector3d>& global_poses,
        std::unordered_map<TrackId, bool>* tracks_to_add) const;

    // Sort the tracks by the number of views that observe them.
    std::vector<TrackId> GetTracksSortedByNumViews(
        const Scene& scene, const View& view,
        const std::unordered_map<TrackId, bool>& existing_tracks) const;

    // Adds all point to camera constraints for a given track.
    void AddTrackToProblem(
        TrackId trackId,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        double point_to_camera_weight,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

    // Adds the points and cameras to parameters groups 0 and 1 respectively.
    // This allows for the Schur-based methods to take advantage of the sparse
    // block structure of the problem by eliminating points first, then cameras.
    // This method is only called if triangulated points are used when solving
    // the problem.
    void AddCamerasAndPointsToParameterGroups(
        std::unordered_map<ViewId, Eigen::Vector3d>* positions);

private:
    const Options options_;
    const Scene& _scene;
    const std::unordered_map<ViewIdPair, ViewPairInfo>* _viewPairs;
    std::set<ViewId> _fixedViewIds;

    std::shared_ptr<RandomNumberGenerator> rng_;
    std::unordered_map<TrackId, Eigen::Vector3d> triangulated_points_;
    std::unique_ptr<ceres::Problem> problem_;
    ceres::Solver::Options solver_options_;

    // friend class EstimatePositionsNonlinearTest;

    // DISALLOW_COPY_AND_ASSIGN(NonlinearPositionEstimator);
};

} // namespace tl
