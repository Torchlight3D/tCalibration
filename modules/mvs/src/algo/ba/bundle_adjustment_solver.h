#pragma once

#include <ceres/ceres.h>
#include <ceres/covariance.h>

#include "bundle_adjustment.h"
#include <AxMVS/Landmark>
#include <AxMVS/Scene>
#include <AxMVS/View>

namespace thoht {

class Camera;
class Scene;
class Track;
class View;

// Brief:
// This class sets up nonlinear optimization problems for bundle adjustment.
//
// Explanation:
// Bundle adjustment problems are set up by adding views and tracks to be
// optimized. Only the views and tracks supplied with addView and addTrack
// will be optimized. All other parameters are held constant.
//
// Note:
// It is required that AddViews is called before AddTracks if any views are
// being optimized.
class BundleAdjuster
{
public:
    // The scene maybe be modified during bundle adjustment.
    BundleAdjuster(const BundleAdjustmentOptions& options, Scene* scene);

    /// Data
    void addView(ViewId id);
    void addTrack(TrackId id);

    void setCameraExtrinsicsConstant(ViewId id);

    /// Actions
    BundleAdjustmentSummary optimize();

    /// Results
    bool calcCovarianceForTrack(TrackId id, Eigen::Matrix3d* covariance);
    bool calcCovarianceForTracks(
        const std::vector<TrackId>& ids,
        std::map<TrackId, Eigen::Matrix3d>* covariances);

    bool calcCovarianceForView(ViewId id, Matrix6d* covariance) const;
    bool calcCovarianceForViews(const std::vector<ViewId>& ids,
                                std::map<ViewId, Matrix6d>& covariances) const;

private:
    void setCameraIntrinsicsParameterization();
    void setCameraExtrinsicsParameterization();

    void setCameraPositionConstant(ViewId id);
    void setCameraOrientationConstant(ViewId id);
    void setTzConstant(ViewId id);

    void setTrackConstant(TrackId id);
    void setTrackVariable(TrackId id);
    void setHomogeneousPointParametrization(TrackId id);

    void setCameraSchurGroups(ViewId id);
    void setTrackSchurGroup(TrackId id);

    void addReprojectionErrorResidual(const Feature& feature, Camera* camera,
                                      Track* track);

    void addInvReprojectionErrorResidual(const Feature& feature,
                                         const Eigen::Vector3d& ref_bearing,
                                         Camera* camera_ref,
                                         Camera* camera_other, Track* track);

    void addPositionPriorErrorResidual(View* view, Camera* camera);

    void addDepthPriorErrorResidual(const Feature& feature, Camera* camera,
                                    Track* track);

    CameraIntrinsics::Ptr intrinsicsOfCamera(CameraId id);

private:
    const BundleAdjustmentOptions options_;
    Scene* scene_;

    std::unique_ptr<ceres::Problem> problem_;
    ceres::Solver::Options solver_options_;

    std::unique_ptr<ceres::LossFunction> loss_function_;
    std::unique_ptr<ceres::LossFunction> depth_prior_loss_function_;

    ceres::ParameterBlockOrdering* parameter_ordering_;

    ceres::Covariance::Options covariance_options_;

    std::unordered_set<ViewId> m_optimizedViewIds;
    std::unordered_set<TrackId> m_optimizedTrackIds;
    std::unordered_set<CameraId> m_optimizedCameraIds;

    // Intrinsics groups that have at least 1 camera marked as "const"
    // during optimization. Only the intrinsics that have no optimized
    // cameras are kept as constant during optimization.
    std::unordered_set<CameraId> m_potentialConstantCameraIds;
};

} // namespace thoht
