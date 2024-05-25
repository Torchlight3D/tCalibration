﻿#include "estimate_calibrated_absolute_pose.h"

#include <tCamera/CameraIntrinsics>
#include <tMath/RANSAC/RansacCreator>
#include <tMath/RANSAC/RansacModelEstimator>
#include <tMvs/BundleAdjustment>
#include <tMvs/Feature>
#include <tMvs/Landmark>
#include <tMvs/PnP/DlsPnP>
#include <tMvs/PnP/P3P>
#include <tMvs/PnP/SQPnP>
#include <tMvs/Scene>
#include <tMvs/View>

namespace tl {

using Eigen::Vector2d;

namespace {

// An estimator for computing the absolute pose from 3 feature
// correspondences. The feature correspondences should be normalized by the
// focal length with the principal point at (0, 0).
class CalibratedAbsolutePoseEstimator
    : public Estimator<Feature2D3D, CalibratedAbsolutePose>
{
    using Base = Estimator<Feature2D3D, CalibratedAbsolutePose>;

public:
    explicit CalibratedAbsolutePoseEstimator(PnPType type = PnPType::KNEIP)
        : Base(), pnp_type_(type)
    {
    }

    // 3 correspondences are needed to determine the absolute pose.
    int SampleSize() const override { return 3; }

    // Estimates candidate absolute poses from corrs.
    bool EstimateModel(
        const std::vector<Feature2D3D>& corrs,
        std::vector<CalibratedAbsolutePose>* poses) const override
    {
        Vector2dList imgPoints(3);
        Vector3dList objPoints(3);
        for (int i = 0; i < 3; ++i) {
            imgPoints[i] = corrs[i].feature;
            objPoints[i] = corrs[i].world_point;
        }

        Matrix3dList rotations;
        Vector3dList translations;
        switch (pnp_type_) {
            case PnPType::KNEIP: {
                if (!P3P(imgPoints, objPoints, rotations, translations)) {
                    return false;
                }
            } break;
            case PnPType::DLS: {
                QuaterniondList quats;
                if (!DlsPnp(imgPoints, objPoints, quats, translations)) {
                    return false;
                }

                std::transform(quats.begin(), quats.end(),
                               std::back_inserter(rotations),
                               [](const auto& q) { return q.matrix(); });
            } break;
            case PnPType::SQPnP: {
                QuaterniondList quats;
                if (!SQPnP(imgPoints, objPoints, quats, translations)) {
                    return false;
                }

                std::transform(quats.begin(), quats.end(),
                               std::back_inserter(rotations),
                               [](const auto& q) { return q.matrix(); });
            } break;
            default:
                break;
        }

        for (size_t i{0}; i < rotations.size(); i++) {
            CalibratedAbsolutePose pose;
            pose.rotation = rotations[i];
            pose.position = -pose.rotation.transpose() * translations[i];
            poses->push_back(pose);
        }

        return !poses->empty();
    }

    bool RefineModel(const std::vector<Feature2D3D>& corrs, double error,
                     CalibratedAbsolutePose* pose) const override
    {
        // Use input pose as initial guess
        Scene scene;
        const auto viewId = scene.addView("0", 0., CameraId{0});
        auto* view = scene.rView(viewId);
        auto& camera = view->rCamera();
        camera.setOrientationFromRotationMatrix(pose->rotation);
        camera.setPosition(pose->position);
        view->setEstimated(true);

        // Setup Scene
        for (const auto& corr : corrs) {
            const auto trackId = scene.addTrack();
            auto* track = scene.rTrack(trackId);
            track->setPosition(corr.world_point.homogeneous());
            track->setEstimated(true);
            scene.addFeature(viewId, trackId, Feature{corr.feature});
        }

        BundleAdjustment::Options opts;
        opts.max_num_iterations = 2;
        opts.use_homogeneous_local_point_parametrization = false;
        opts.intrinsics_to_optimize = OptimizeIntrinsicsType::None;
        opts.loss_function_type = LossFunctionType::Huber;
        opts.robust_loss_width = error * 1.5;

        const auto summary = BundleAdjustView(opts, viewId, &scene);

        pose->position = camera.position();
        pose->rotation = camera.orientationAsRotationMatrix();

        return summary.success && summary.final_cost < summary.initial_cost;
    }

    // The error for a correspondences given an absolute pose. This is the
    // squared reprojection error.
    double Error(const Feature2D3D& corr,
                 const CalibratedAbsolutePose& pose) const override
    {
        // x = R * (X - t)
        const Vector2d reprojected =
            (pose.rotation * (corr.world_point - pose.position)).hnormalized();
        return (reprojected - corr.feature).squaredNorm();
    }

private:
    PnPType pnp_type_;
};

} // namespace

bool EstimateCalibratedAbsolutePose(
    const SacParameters& sac_params, RansacType ransac_type, PnPType pnp_type,
    const std::vector<Feature2D3D>& normalized_correspondences,
    CalibratedAbsolutePose* absolute_pose, SacSummary* ransac_summary)
{
    CalibratedAbsolutePoseEstimator estimator{pnp_type};
    auto ransac = createRansac(ransac_type, sac_params, estimator);

    return ransac->Estimate(normalized_correspondences, absolute_pose,
                            ransac_summary);
}

} // namespace tl
