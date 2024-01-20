#include "camera_pose_calibration.h"

#include <thread>
#include <unordered_map>

#include <json/json.hpp>

#include <tTarget/CalibBoardBase>
#include <tCore/ContainerUtils>
#include <tCore/TimeUtils>
#include <tVision/EigenCVUtils>
#include <tMath/SampleConsensusEstimator>
#include <tMvs/BundleAdjustment>
#include <tMvs/EstimateCalibratedAbsolutePose>
#include <tMvs/FeatureCorrespondence>
#include <tMvs/Landmark>
#include <tMvs/View>
#include <tSolver/LossFunctionCreator>

#include "util_scene.h"

namespace tl {

///------- PoseEstimator::Impl implementation starts from here
class CameraPoseCalibration::Impl
{
public:
    Impl();

    void init(const Options& options);

    bool estimatePosePinhole(
        ViewId viewId,
        const std::vector<FeatureCorrespondence2D3D>& correspondences_undist,
        const std::vector<TrackId>& trackIds);

public:
    Options m_opts;
    Scene::Ptr m_scene;
    std::unordered_map<TrackId, size_t> m_trackIdToObsCount;
    double m_maxRPE = 0.;
    double m_sacErrThreshold = 0.;
};

CameraPoseCalibration::Impl::Impl() : m_scene(std::make_shared<Scene>()) {}

void CameraPoseCalibration::Impl::init(const Options& options)
{
    m_opts = options;
}

bool CameraPoseCalibration::Impl::estimatePosePinhole(
    ViewId viewId,
    const std::vector<FeatureCorrespondence2D3D>& correspondences,
    const std::vector<TrackId>& trackIds)
{
    SacParameters ransacParams;
    ransacParams.failure_probability = 0.001;
    ransacParams.use_mle = true;
    ransacParams.max_iterations = 1000;
    ransacParams.min_iterations = 10;
    ransacParams.error_thresh = m_sacErrThreshold;

    constexpr RansacType ransacType = RansacType::RANSAC;
    constexpr PnPType pnpType = PnPType::DLS;

    CalibratedAbsolutePose pose;
    SacSummary ransacSummary;
    EstimateCalibratedAbsolutePose(ransacParams, ransacType, pnpType,
                                   correspondences, &pose, &ransacSummary);

    constexpr int kMinInlierCount{6};
    if (ransacSummary.inliers.size() < kMinInlierCount) {
        return false;
    }

    auto* view = m_scene->rView(viewId);
    view->setEstimated(true);

    auto& cam = view->rCamera();
    cam.setPosition(pose.position);
    cam.setOrientationFromRotationMatrix(pose.rotation);

    for (const auto& inlier : ransacSummary.inliers) {
        m_scene->addFeature(viewId, trackIds[inlier],
                            Feature(correspondences[inlier].feature));
    }

    BundleAdjustmentOptions ba_opts;
    ba_opts.loss_function_type = LossFunctionType::Huber;
    ba_opts.robust_loss_width = 1.345;
    ba_opts.intrinsics_to_optimize = OptimizeIntrinsicsType::None;
    auto ba_summary = BundleAdjustView(ba_opts, viewId, m_scene.get());

    return ba_summary.success;
}

///------- PoseEstimator starts from here
CameraPoseCalibration::CameraPoseCalibration(const Options& options)
    : d(std::make_unique<Impl>())
{
    d->init(options);
}

CameraPoseCalibration::~CameraPoseCalibration() = default;

const CameraPoseCalibration::Options& CameraPoseCalibration::options() const
{
    return d->m_opts;
}

CameraPoseCalibration::Options& CameraPoseCalibration::rOptions()
{
    return d->m_opts;
}

void CameraPoseCalibration::setupScene(const CalibBoardBase& board)
{
    mvs::setupBoardInScene(d->m_scene, board);
}

bool CameraPoseCalibration::calibrate(const StampedTargetDetections& detections,
                                      const Camera& camera, CameraId camId)
{
    if (!camera.calibrated()) {
        LOG(ERROR) << "Failed to estimate camera poses: "
                      "Camera "
                   << camId << " is not calibrated.";
        return false;
    }

    LOG(INFO) << "======= "
                 "Start camera pose estimation (Camera "
              << camId
              << ")"
                 " =======";

    int w = camera.imageWidth();
    int h = camera.imageHeight();
    const double diagonal = std::sqrt(w * w + h * h);

    // Set error thresh 0.4% from image size and normalize
    constexpr double kErrorRatio{0.004};
    d->m_maxRPE = kErrorRatio * camera.imageHeight();
    d->m_sacErrThreshold = d->m_maxRPE / diagonal;

    LOG(INFO) << "Setting max reprojection error to: " << d->m_maxRPE;

    // Init the number of observations per point to zero.
    // It might happen, that some point are not seen at all and
    // then BA will crash as no observations are available
    for (const auto& trackId : d->m_scene->trackIds()) {
        d->m_trackIdToObsCount[trackId] = 0;
    }

    LOG(INFO) << "Estimating " << detections.size() << " detection views.";

    double totalViewRPE{0.0};
    int processedViewCount{0};
    for (const auto& detection : detections) {
        const double& timestamp = detection.t;

        const auto cornerCount = detection.cornerCount();
        std::vector<TrackId> obsTrackIds;
        std::vector<FeatureCorrespondence2D3D> correspondences;
        for (size_t i{0}; i < cornerCount; ++i) {
            const auto trackId = detection.cornerIds()[i];
            obsTrackIds.push_back(trackId);

            Eigen::Vector3d undist_pt = camera.pixelToNormalizedCoordinates(
                cvPoint2ToEigen(detection.corners()[i]));
            undist_pt /= undist_pt[2];

            const Eigen::Vector4d track =
                d->m_scene->track(trackId)->position();

            FeatureCorrespondence2D3D corr;
            corr.world_point = track.hnormalized();
            corr.feature = undist_pt.head<2>();
            correspondences.push_back(corr);
        }

        if (correspondences.size() < d->m_opts.min_num_points_) {
            LOG(INFO) << "Skip view at timestamp " << timestamp
                      << "s: "
                         "Not enough points found.";
            continue;
        }

        // Add this view
        const auto viewName = mvs::makeUniqueViewName(camId, timestamp);
        const auto viewId = d->m_scene->addView(viewName, camId, timestamp);

        // We only use the pose of the camera, and don't care about
        // intrinsics, so use default Pinhole camera
        auto& cam = d->m_scene->rView(viewId)->rCamera();
        cam.setCameraIntrinsicsModel(CameraIntrinsics::Type::Pinhole);
        cam.setFocalLength(1.);
        cam.setPrincipalPoint(0., 0.);
        cam.setImageSize(1., 1.);
        if (!d->estimatePosePinhole(viewId, correspondences, obsTrackIds)) {
            LOG(INFO) << "Camera pose estimation failed for view at timestamp "
                      << timestamp << "s from " << correspondences.size()
                      << " correspondences.";
            d->m_scene->removeView(viewId);
            continue;
        }

        for (const auto& trackId : d->m_scene->view(viewId)->trackIds()) {
            d->m_trackIdToObsCount[trackId] += 1;
        }

        const double viewRPE = d->m_scene->calcViewReprojectionError(viewId);
        if (viewRPE > d->m_maxRPE) {
            LOG(INFO) << "Removing view " << viewId
                      << ": "
                         "Large reprojection error "
                      << viewRPE << "> max " << d->m_maxRPE << " px";
            d->m_scene->removeView(viewId);
        }

        totalViewRPE += viewRPE;
        ++processedViewCount;
    }

    LOG(INFO) << "Total RPE " << totalViewRPE << " from " << processedViewCount
              << " processed views.";

    return true;
}

void CameraPoseCalibration::optimizeAllPoses()
{
    BundleAdjustmentOptions ba_opts;
    ba_opts.loss_function_type = LossFunctionType::Huber;
    ba_opts.robust_loss_width = 1.345;
    ba_opts.intrinsics_to_optimize = OptimizeIntrinsicsType::None;
    ba_opts.constant_camera_orientation = false;
    ba_opts.constant_camera_position = false;
    ba_opts.verbose = false;

    LOG(INFO) << "======= "
                 "Start optimizing all estimated view poses"
                 " =======";
    for (const auto& viewId : d->m_scene->viewIds()) {
        BundleAdjustView(ba_opts, viewId, d->m_scene.get());
    }
}

void CameraPoseCalibration::optimizeBoardPoints()
{
    // Only use track ids that have actually been observed more than minimum
    // times
    std::vector<TrackId> optimizeTrackIds;
    for (const auto& [trackId, obsCount] : d->m_trackIdToObsCount) {
        if (obsCount > d->m_opts.min_num_obs_for_optim_) {
            optimizeTrackIds.push_back(trackId);
        }
    }

    BundleAdjustmentOptions ba_opts;
    ba_opts.loss_function_type = LossFunctionType::Huber;
    ba_opts.robust_loss_width = 1.345;
    ba_opts.intrinsics_to_optimize = OptimizeIntrinsicsType::None;
    ba_opts.constant_camera_orientation = true;
    ba_opts.constant_camera_position = true;
    ba_opts.verbose = true;

    LOG(INFO) << "======= "
                 "Start optimizing board points"
                 " =======";

    std::map<TrackId, Eigen::Matrix3d> covariances;
    double variance;
    BundleAdjustTracks(ba_opts, optimizeTrackIds, d->m_scene.get(),
                       &covariances, &variance);
    LOG(INFO) << "Empirical variance factor after board point optimization: "
              << variance;

    Eigen::Vector3d mean_std(0.0, 0.0, 0.0);
    for (const auto& cov : covariances) {
        Eigen::Vector3d stddev = cov.second.diagonal().array().sqrt() * 1e3;
        mean_std += stddev;
        LOG(INFO) << "Track Id: " << cov.first
                  << " std dev: " << stddev.transpose() << " mm";
    }
    mean_std /= covariances.size();
    LOG(INFO) << "Mean board point standard deviation after optimization: "
              << mean_std.transpose() << " mm";
}

void CameraPoseCalibration::filterBadPoses()
{
    // Sometimes it happens that poses are far away or on the wrong side of the
    // calibration board
    std::vector<double> z_values;
    for (const auto& viewId : d->m_scene->viewIds()) {
        const auto pos = d->m_scene->view(viewId)->camera().position();
        z_values.push_back(pos[2]);
    }

    // Get median distance to board
    double median_z = utils::FindMedian(z_values);
    for (const auto& viewId : d->m_scene->viewIds()) {
        const auto pos = d->m_scene->view(viewId)->camera().position();
        double diff = pos[2] - median_z;
        if (std::abs(diff) > std::abs(median_z)) {
            LOG(INFO) << "Remove view " << viewId
                      << ": "
                         "Large z coordinate "
                      << pos[2] << "> median z " << median_z;
            d->m_scene->removeView(viewId);
        }
    }
}

Scene::Ptr CameraPoseCalibration::scene() const { return d->m_scene; }

bool CameraPoseCalibration::setFromJson(const std::string& json)
{
    //
    return false;
}

void CameraPoseCalibration::toJsonString(std::string& json) const
{
    //
}

} // namespace tl
