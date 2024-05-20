#include "estimate_calibrated_absolute_pose.h"

#include <tCamera/CameraIntrinsics>
#include <tMath/RANSAC/RansacCreator>
#include <tMath/RANSAC/RansacModelEstimator>
#include <tMvs/BundleAdjustment>
#include <tMvs/FeatureCorrespondence>
#include <tMvs/Landmark>
#include <tMvs/PnP/DlsPnP>
#include <tMvs/PnP/P3P>
#include <tMvs/PnP/SQPnP>
#include <tMvs/Scene>
#include <tMvs/View>

namespace tl {
namespace {

// An estimator for computing the absolute pose from 3 feature
// correspondences. The feature correspondences should be normalized by the
// focal length with the principal point at (0, 0).
class CalibratedAbsolutePoseEstimator
    : public Estimator<FeatureCorrespondence2D3D, CalibratedAbsolutePose>
{
public:
    explicit CalibratedAbsolutePoseEstimator(PnPType type = PnPType::KNEIP)
        : pnp_type_(type)
    {
        ba_opts_.max_num_iterations = 2;
        ba_opts_.use_homogeneous_local_point_parametrization = false;
        ba_opts_.intrinsics_to_optimize = OptimizeIntrinsicsType::None;
    }

    // 3 correspondences are needed to determine the absolute pose.
    double SampleSize() const override { return 3; }

    // Estimates candidate absolute poses from correspondences.
    bool EstimateModel(
        const std::vector<FeatureCorrespondence2D3D>& correspondences,
        std::vector<CalibratedAbsolutePose>* absolute_poses) const override
    {
        Vector2dList features(3);
        Vector3dList world_points(3);
        for (int i = 0; i < 3; ++i) {
            features[i] = correspondences[i].feature;
            world_points[i] = correspondences[i].world_point;
        }

        Matrix3dList rotations;
        Vector3dList translations;
        switch (pnp_type_) {
            case PnPType::KNEIP: {
                if (!P3P(features, world_points, rotations, translations)) {
                    return false;
                }
            } break;
            case PnPType::DLS: {
                QuaterniondList quats;
                if (!DlsPnp(features, world_points, quats, translations)) {
                    return false;
                }

                std::transform(quats.begin(), quats.end(),
                               std::back_inserter(rotations),
                               [](const auto& q) { return q.matrix(); });
            } break;
            case PnPType::SQPnP: {
                QuaterniondList quats;
                if (!SQPnP(features, world_points, quats, translations)) {
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
            absolute_poses->push_back(pose);
        }

        return absolute_poses->size() > 0;
    }

    bool RefineModel(
        const std::vector<FeatureCorrespondence2D3D>& correspondences,
        CalibratedAbsolutePose* absolute_pose) const override
    {
        Scene scene;
        const auto viewId = scene.addView("0", 0, 0.0);
        auto* view = scene.rView(viewId);
        view->setEstimated(true);

        auto& camera = view->rCamera();
        camera.setOrientationFromRotationMatrix(absolute_pose->rotation);
        camera.setPosition(absolute_pose->position);
        for (const auto& corr : correspondences) {
            const auto trackId = scene.addTrack();
            auto* track = scene.rTrack(trackId);
            track->setEstimated(true);
            track->setPosition(corr.world_point.homogeneous());
            scene.addFeature(viewId, trackId, Feature(corr.feature));
        }

        BundleAdjustmentSummary ba_summary =
            BundleAdjustView(ba_opts_, viewId, &scene);

        absolute_pose->position = camera.position();
        absolute_pose->rotation = camera.orientationAsRotationMatrix();
        return ba_summary.final_cost < ba_summary.initial_cost &&
               ba_summary.success;
    }

    // The error for a correspondences given an absolute pose. This is the
    // squared reprojection error.
    double Error(const FeatureCorrespondence2D3D& correspondence,
                 const CalibratedAbsolutePose& absolute_pose) const override
    {
        // The reprojected point is computed as R * (X - c) where R is the
        // camera rotation, c is the position, and X is the 3D point.
        const Eigen::Vector2d reprojected_feature =
            (absolute_pose.rotation *
             (correspondence.world_point - absolute_pose.position))
                .hnormalized();
        return (reprojected_feature - correspondence.feature).squaredNorm();
    }

private:
    PnPType pnp_type_;
    BundleAdjustmentOptions ba_opts_;
};

} // namespace

bool EstimateCalibratedAbsolutePose(
    const SacParameters& sac_params, RansacType ransac_type, PnPType pnp_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences,
    CalibratedAbsolutePose* absolute_pose, SacSummary* ransac_summary)
{
    CalibratedAbsolutePoseEstimator estimator{pnp_type};
    auto ransac = createRansac(ransac_type, sac_params, estimator);

    return ransac->Estimate(normalized_correspondences, absolute_pose,
                            ransac_summary);
}

} // namespace tl
