#include "cameraintrinsicscalibration.h"

#include <map>
#include <thread>

#include <opencv2/core.hpp>

#include <json/json.hpp>
#include <magic_enum/magic_enum.hpp>

#include <tCamera/Camera>
#include <tCamera/DivisionUndistortionCameraModel>
#include <tCamera/DoubleSphereCameraModel>
#include <tCamera/ExtendedUnifiedCameraModel>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCore/TimeUtils>
#include <tMath/Ransac/SampleConsensus>
#include <tMvs/BundleAdjustment>
#include <tMvs/Feature>
#include <tMvs/Landmark>
#include <tMvs/Scene>
#include <tMvs/View>
#include <tVision/EigenCVUtils>

#include "util_scene.h"
#include "initializeintrinsics.h"

namespace tl {

///------- CalibSampleStatistic starts from here
// WARNING: Not ready
class CalibSampleStatistic
{
public:
    explicit CalibSampleStatistic(const cv::Size& imgSize = {},
                                  double centerRatio = 0.7);

    /// Properties
    void setCenterRatio(double ratio);

    /// Data
    void addSample(const cv::Point& point);

    /// Actions
    bool satisfy() const;

    /// Debug
    const cv::Mat& distribution() const;

private:
    enum
    {
        Anchor1,
        Anchor2,

        AnchorSize,
    };

    inline static constexpr uchar kMark = 255;

    cv::Mat distribution_;
    std::array<double, AnchorSize> x_anchors_, y_anchors_;
};

CalibSampleStatistic::CalibSampleStatistic(const cv::Size& imgSize,
                                           double centerRatio)
{
    if (!imgSize.empty()) {
        distribution_ = cv::Mat::zeros(imgSize, CV_8UC1);
        setCenterRatio(centerRatio);
    }
}

// When center region is fixed, corner and edge regions are fixed too.
//    x0  x1         x2  x3
// y0 +---+----------+---+
//    |   |          |   |
// y1 +---+----------+---+
//    |   |          |   |
//    |   |  Center  |   |
//    |   |          |   |
// y2 +---+----------+---+
//    |   |          |   |
// y3 +---+----------+---+
void CalibSampleStatistic::setCenterRatio(double ratio)
{
    if (distribution_.empty() || ratio > 1. || ratio <= 0.) {
        return;
    }

    const auto size = distribution_.size();

    const double ratio1 = (1. - ratio) * 0.5;
    const double ratio2 = (1. + ratio) * 0.5;
    x_anchors_[Anchor1] = ratio1 * size.width;
    x_anchors_[Anchor2] = ratio2 * size.width;
    y_anchors_[Anchor1] = ratio1 * size.height;
    y_anchors_[Anchor2] = ratio2 * size.height;
}

void CalibSampleStatistic::addSample(const cv::Point& point)
{
    if (distribution_.empty()) {
        return;
    }

    distribution_.at<uchar>(point) = kMark;
}

bool CalibSampleStatistic::satisfy() const
{
    if (distribution_.empty()) {
        return false;
    }

    const cv::Rect imgRect{cv::Point{}, distribution_.size()};
    const int w = imgRect.width;
    const int h = imgRect.height;

    const auto& x1 = x_anchors_[Anchor1];
    const auto& x2 = x_anchors_[Anchor2];
    const auto& y1 = y_anchors_[Anchor1];
    const auto& y2 = y_anchors_[Anchor2];

    const double w0 = x1;
    const double w1 = x2 - x1;
    const double w2 = w - x2;
    const double h0 = y1;
    const double h1 = y2 - y1;
    const double h2 = h - y2;

    const cv::Rect tlRect{cv::Point2d{0., 0.}, cv::Size2d{w0, h0}};
    const cv::Rect topRect{cv::Point2d{x1, 0.}, cv::Size2d{w1, h0}};
    const cv::Rect trRect{cv::Point2d{x2, 0.}, cv::Size2d{w2, h0}};
    const cv::Rect leftRect{cv::Point2d{0., y1}, cv::Size2d{w0, h1}};
    const cv::Rect centerRect{cv::Point2d{x1, y1}, cv::Size2d{w1, h1}};
    const cv::Rect rightRect{cv::Point2d{x2, y1}, cv::Size2d{w2, h1}};
    const cv::Rect blRect{cv::Point2d{0., y2}, cv::Size2d{w0, h2}};
    const cv::Rect bottomRect{cv::Point2d{x1, y2}, cv::Size2d{w1, h2}};
    const cv::Rect brRect{cv::Point2d{x2, y2}, cv::Size2d{w2, h2}};

    auto pointCount = [](const cv::Mat& img, const cv::Rect& roi) -> int {
        const auto scalar = cv::sum(img(roi));
        return static_cast<int>(scalar(0) / kMark);
    };

    const auto tlCount = pointCount(distribution_, tlRect);
    const auto topCount = pointCount(distribution_, topRect);
    const auto trCount = pointCount(distribution_, trRect);
    const auto leftCount = pointCount(distribution_, leftRect);
    [[maybe_unused]] const auto centerCount =
        pointCount(distribution_, centerRect);
    const auto rightCount = pointCount(distribution_, rightRect);
    const auto blCount = pointCount(distribution_, blRect);
    const auto bottomCount = pointCount(distribution_, bottomRect);
    const auto brCount = pointCount(distribution_, brRect);

    // Test Rule 1: All corners and edges should have more observation points
    // than the fixed minimum.
    constexpr int kMinCorner = 15;

    const auto cornersCount = {tlCount, trCount, blCount, brCount};
    const bool cornerPass = std::all_of(
        cornersCount.begin(), cornersCount.end(),
        [min = kMinCorner](const auto& count) { return count >= min; });

    constexpr int kMinEdge = 20;

    const auto edgesCount = {topCount, leftCount, rightCount, bottomCount};
    const bool edgePass = std::all_of(
        edgesCount.begin(), edgesCount.end(),
        [min = kMinEdge](const auto& count) { return count >= min; });

    return cornerPass && edgePass;
}

const cv::Mat& CalibSampleStatistic::distribution() const
{
    return distribution_;
}

///------- CameraIntrinsicsCalibration::Impl starts from here
class CameraIntrinsicsCalibration::Impl
{
public:
    Impl();

    void init(const Options& opts);

    void updateConfigs();

    // FIXME: Duplicated code!!!
    bool initializeView(const StampedTargetDetection& detection,
                        CameraId id = {});
    void initializeViews(const StampedTargetDetections& detections,
                         CameraId id = {});

    CameraIntrinsicsCalibration::Summary::ErrorCode startBA(CameraId id = {});

    ViewId addViewToScene(const Eigen::Matrix3d& orientation,
                          const Eigen::Vector3d& position, double focalLength,
                          double distortion, double timestamp,
                          CameraId id = {});

    // TODO: Make generic function to accept rejection rule
    void removeViewsByReprojectionError(double maxRPE, CameraId id = {});
    void removeViewsByGeometry(CameraId id = {});

public:
    Scene::Ptr m_scene; // own
    Options m_opts;
    // For both incremental and batch interfaces
    SacParameters m_ransacParams;
    Eigen::Vector2d m_center;
    eigen_map<CameraId, Vector3dList> m_existedViewPoses;
    CalibSampleStatistic m_sampleStats;
};

CameraIntrinsicsCalibration::Impl::Impl() : m_scene(std::make_shared<Scene>())
{
}

void CameraIntrinsicsCalibration::Impl::init(const Options& opts)
{
    m_opts = opts;

    m_ransacParams.failure_probability = 0.001;
    m_ransacParams.use_mle = true;
    m_ransacParams.max_iterations = 1000;
    m_ransacParams.min_iterations = 5;
    m_ransacParams.error_thresh = 3.;

    m_center = {0., 0.};

    updateConfigs();
}

void CameraIntrinsicsCalibration::Impl::updateConfigs()
{
    constexpr double kErrorRatio{0.003};
    m_ransacParams.error_thresh = kErrorRatio * m_opts.imageHeight;
    m_center = {m_opts.imageWidth / 2., m_opts.imageHeight / 2.};
    m_sampleStats =
        CalibSampleStatistic{{m_opts.imageWidth, m_opts.imageHeight}};
}

bool CameraIntrinsicsCalibration::Impl::initializeView(
    const StampedTargetDetection& detection, CameraId camId)
{
    if (!detection.valid() || detection.cornerCount() == 0) {
        LOG(ERROR) << "Failed to initialize view: "
                      "Invalid detection from camera "
                   << camId;
        return false;
    }

    const auto& corners = detection.corners();
    const auto& cornerIds = detection.cornerIds();
    const auto& timestamp = detection.t;
    const auto cornerCount = detection.cornerCount();
    VLOG(-1) << "Initializing view at " << timestamp
             << "s: "
                "corner count of  "
             << cornerCount;

    // Cache
    Vector2dList features;
    features.reserve(cornerCount);
    std::vector<Feature2D3D> corrs(cornerCount);
    for (int i{0}; i < cornerCount; ++i) {
        const auto feature = cvPoint2ToEigen(corners[i]);
        features.push_back(feature);
        corrs[i].feature = feature - m_center;
        corrs[i].world_point =
            m_scene->track(cornerIds[i])->position().hnormalized();
    }

    SacSummary ransacSummary;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    double focalLength{0.};
    double radialDistortion{0.};

    bool initialized{false};
    switch (m_opts.intrinsicsType) {
        case CameraIntrinsicsType::Pinhole:
        case CameraIntrinsicsType::PinholeRadialTangential:
            initialized =
                initializePinholeCamera(corrs, m_ransacParams, ransacSummary,
                                        rotation, position, focalLength);
            break;
        case CameraIntrinsicsType::DivisionUndistortion:
            initialized = initializeRadialUndistortionCamera(
                corrs, m_ransacParams, ransacSummary, m_opts.imageWidth,
                rotation, position, focalLength, radialDistortion);
            break;
        case CameraIntrinsicsType::Omnidirectional:
            // TODO: Not ready
            // initialized = initializeOmnidirectional(
            //     corrs, m_ransacParams, ransacSummary, m_opts.imageWidth,
            //     rotation, position, focalLength);
            // break;
        case CameraIntrinsicsType::DoubleSphere:
            // TODO: Not ready
            // initialized = initializeDoubleSphereModel(
            //     corrs, board_pt3_ids, cv::Size(9, 7), m_ransacParams,
            //     img_size, ransacSummary, rotation, position, focalLength);
            // break;
        default:
            initialized = initializeRadialUndistortionCamera(
                corrs, m_ransacParams, ransacSummary, m_opts.imageWidth,
                rotation, position, focalLength, radialDistortion);
            break;
    }

    if (!initialized) {
        VLOG(-1) << "Skip this view: "
                    "Failed to initialize view.";
        return false;
    }

    auto& existedViewPoses = m_existedViewPoses[camId];

    // Strategy: Keep minimum distance among views
    if (m_opts.checkSampleDistance) {
        const bool closeToExistedView = std::any_of(
            existedViewPoses.cbegin(), existedViewPoses.cend(),
            [thisPos = position, minDist = m_opts.sampleDistance](
                const auto& pos) { return (pos - thisPos).norm() < minDist; });
        if (closeToExistedView) {
            VLOG(-1) << "Skip this view: "
                        "Too close to existed views.";
            return false;
        }
    }

    const auto viewId = addViewToScene(rotation, position, focalLength,
                                       radialDistortion, timestamp, camId);
    if (viewId == kInvalidViewId) {
        VLOG(-1) << "Skip this view: "
                    "View UID already exists.";
        return false;
    }

    for (size_t i{0}; i < cornerCount; ++i) {
        m_scene->addFeature(viewId, cornerIds[i], features[i]);
        m_sampleStats.addSample(corners[i]);
    }

    existedViewPoses.push_back(position);

    return true;
}

void CameraIntrinsicsCalibration::Impl::initializeViews(
    const StampedTargetDetections& detections, CameraId camId)
{
    LOG(INFO) << "======= "
                 "Start initializing views, use "
              << magic_enum::enum_name(m_opts.intrinsicsType)
              << " as camera model."
                 " =======";

    const auto detectionCount = detections.size();

    auto& existedViewPoses = m_existedViewPoses[camId];
    existedViewPoses.clear();

    int initializedViewCount{0};
    for (const auto& detection : detections) {
        if (!detection.valid()) {
            continue;
        }

        const auto& corners = detection.corners();
        const auto& cornerIds = detection.cornerIds();
        const auto& timestamp = detection.t;
        const auto cornerCount = detection.cornerCount();
        VLOG(-1) << "Initializing view at " << timestamp
                 << "s: "
                    "corner count of  "
                 << cornerCount;

        // Cache
        Vector2dList features;
        features.reserve(cornerCount);
        std::vector<Feature2D3D> correspondences(cornerCount);
        for (int i{0}; i < cornerCount; ++i) {
            const auto feature = cvPoint2ToEigen(corners[i]);
            features.push_back(feature);
            correspondences[i].feature = feature - m_center;
            correspondences[i].world_point =
                m_scene->track(cornerIds[i])->position().hnormalized();
        }

        SacSummary ransacSummary;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d position;
        double focalLength{0.};
        double radialDistortion{0.};

        bool initialized{false};
        switch (m_opts.intrinsicsType) {
            case CameraIntrinsicsType::Pinhole:
            case CameraIntrinsicsType::PinholeRadialTangential:
                initialized = initializePinholeCamera(
                    correspondences, m_ransacParams, ransacSummary, rotation,
                    position, focalLength);
                break;
            case CameraIntrinsicsType::DivisionUndistortion:
                initialized = initializeRadialUndistortionCamera(
                    correspondences, m_ransacParams, ransacSummary,
                    m_opts.imageWidth, rotation, position, focalLength,
                    radialDistortion);
                break;
            default:
                initialized = initializeRadialUndistortionCamera(
                    correspondences, m_ransacParams, ransacSummary,
                    m_opts.imageWidth, rotation, position, focalLength,
                    radialDistortion);
                // TODO: Not ready
                //                initialized = initializeDoubleSphereModel(
                //                    correspondences, board_pt3_ids,
                //                    cv::Size(9, 7), d->ransac_params_,
                //                    img_size, ransac_summary, rotation,
                //                    position, focal_length, d->verbose_);
                break;
        }

        if (!initialized) {
            VLOG(-1) << "Skip this view: Failed to initialize view.";
            continue;
        }

        if (m_opts.checkSampleDistance) {
            // TODO: Use kd-tree
            bool closeToExistedView =
                std::any_of(existedViewPoses.begin(), existedViewPoses.end(),
                            [thisPos = position,
                             minDist = m_opts.sampleDistance](const auto& pos) {
                                return (pos - thisPos).norm() < minDist;
                            });

            if (closeToExistedView) {
                VLOG(-1) << "Skip this view: Too close to existed views.";
                continue;
            }
        }

        const auto viewId = addViewToScene(rotation, position, focalLength,
                                           radialDistortion, timestamp, camId);
        if (viewId == kInvalidViewId) {
            VLOG(-1) << "Skip this view: View UID already exists.";
            continue;
        }

        for (size_t i{0}; i < cornerCount; ++i) {
            m_scene->addFeature(viewId, cornerIds[i], features[i]);
            m_sampleStats.addSample(corners[i]);
        }

        existedViewPoses.push_back(position);

        ++initializedViewCount;
        LOG_EVERY_N(INFO, 100) << initializedViewCount << " in "
                               << detectionCount << " views are initialized.";
    }
}

CameraIntrinsicsCalibration::Summary::ErrorCode
CameraIntrinsicsCalibration::Impl::startBA(CameraId camId)
{
    using ErrorCode = CameraIntrinsicsCalibration::Summary::ErrorCode;

    const auto initViewCount = m_scene->viewCount(camId);
    if (initViewCount < m_opts.minViewCount) {
        LOG(ERROR) << "Camera calibration BA failed: "
                      "Not enough views to start a calibration. "
                   << initViewCount << "/" << m_opts.minViewCount
                   << "(current/minimum)";
        return ErrorCode::NotEnoughData;
    }

    LOG(INFO) << "======= "
                 "Start calibration BA, use "
              << initViewCount
              << " views."
                 " =======";

    BundleAdjustment::Options ba_options;
    ba_options.verbose = true;
    ba_options.loss_function_type = LossFunctionType::Cauchy;
    ba_options.robust_loss_width = 1.0;
    ba_options.num_threads = std::thread::hardware_concurrency();
    ba_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ba_options.function_tolerance = 1e-3;
    ba_options.gradient_tolerance = 1e-3;
    ba_options.parameter_tolerance = 1e-3;
    ba_options.max_trust_region_radius = 1e4;

    // 1. Optimize focal length (xi if neccessary), principal point and aspect
    // ratio, while keeping others fixed
    ba_options.constant_camera_orientation = false;
    ba_options.constant_camera_position = false;
    ba_options.intrinsics_to_optimize = OptimizeIntrinsicsType::PrincipalPoint |
                                        OptimizeIntrinsicsType::FocalLength |
                                        OptimizeIntrinsicsType::AspectRatio;

    VLOG(-1) << "Optimizing focal length and principal point.";
    {
        const auto sharedCamViewIds = m_scene->sharedCameraViewIds(camId);
        [[maybe_unused]] auto summary = BundleAdjustViews(
            ba_options, {sharedCamViewIds.begin(), sharedCamViewIds.end()},
            m_scene.get());
    }

    removeViewsByReprojectionError(2., camId);
    removeViewsByGeometry(camId);

    if (m_scene->viewCount(camId) < m_opts.minViewCount) {
        LOG(ERROR)
            << "Camera calibration BA failed: "
               "Not enough views left after optimizing principal points. "
            << m_scene->viewCount(camId) << "/" << m_opts.minViewCount
            << "(current/minimum)";
        return ErrorCode::NotEnoughValidData;
    }

    // 2. Optimize distortion after remove outliers, while keeping others fixed
    ba_options.constant_camera_orientation = true;
    ba_options.constant_camera_position = true;
    ba_options.intrinsics_to_optimize =
        OptimizeIntrinsicsType::RadialDistortion |
        OptimizeIntrinsicsType::TangentialDistortion;

    VLOG(-1) << "Optimizing distortion.";
    {
        const auto sharedCamViewIds = m_scene->sharedCameraViewIds(camId);
        [[maybe_unused]] auto summary = BundleAdjustViews(
            ba_options, {sharedCamViewIds.begin(), sharedCamViewIds.end()},
            m_scene.get());
    }

    // 3. Full optimization, except skew
    ba_options.constant_camera_orientation = false;
    ba_options.constant_camera_position = false;
    ba_options.intrinsics_to_optimize = OptimizeIntrinsicsType::All;

    VLOG(-1) << "Optimizing all intrinsic parameters.";
    {
        const auto sharedCamViewIds = m_scene->sharedCameraViewIds(camId);
        [[maybe_unused]] auto summary = BundleAdjustViews(
            ba_options, {sharedCamViewIds.begin(), sharedCamViewIds.end()},
            m_scene.get());
    }

    // 4. Optimize board points
    if (m_opts.optimizeBoardPoints) {
        ba_options.use_homogeneous_local_point_parametrization = false;

        VLOG(-1) << "Optimizing board points.";
        {
            [[maybe_unused]] auto trackSummary = BundleAdjustTracks(
                ba_options, m_scene->trackIds(), m_scene.get());

            const auto sharedCamViewIds = m_scene->sharedCameraViewIds(camId);
            [[maybe_unused]] auto viewSummary = BundleAdjustViews(
                ba_options, {sharedCamViewIds.begin(), sharedCamViewIds.end()},
                m_scene.get());
        }
    }

    const auto sharedCamViewIds = m_scene->sharedCameraViewIds(camId);
    m_scene->rView(*sharedCamViewIds.begin())->rCamera().setCalibrated(true);

    return ErrorCode::Success;
}

ViewId CameraIntrinsicsCalibration::Impl::addViewToScene(
    const Eigen::Matrix3d& orientation, const Eigen::Vector3d& position,
    double focalLength, double distortion, double timestamp, CameraId camId)
{
    // Add view
    const auto viewName = mvs::makeUniqueViewName(camId, timestamp);
    const auto viewId = m_scene->addView(viewName, timestamp, camId);
    if (viewId == kInvalidViewId) {
        return kInvalidViewId;
    }

    auto* view = m_scene->rView(viewId);
    view->setEstimated(true);

    // Initialize camera
    auto& cam = view->rCamera();
    cam.setImageSize(m_opts.imageWidth, m_opts.imageHeight);

    // Extrinsic
    cam.setPosition(position);
    cam.setOrientationFromRotationMatrix(orientation);

    // Intrinsics
    cam.setCameraIntrinsicsModel(m_opts.intrinsicsType);
    cam.setFocalLength(focalLength);
    cam.setPrincipalPoint(m_opts.imageWidth / 2., m_opts.imageHeight / 2.);

    auto intrinsics = cam.cameraIntrinsics();
    switch (m_opts.intrinsicsType) {
        case CameraIntrinsicsType::DivisionUndistortion:
            intrinsics->setParameter(DivisionUndistortionCameraModel::K,
                                     distortion);
            break;
        case CameraIntrinsicsType::DoubleSphere:
            intrinsics->setParameter(DoubleSphereCameraModel::Xi, -0.25);
            intrinsics->setParameter(DoubleSphereCameraModel::Alpha, 0.5);
            break;
        case CameraIntrinsicsType::ExtendedUnified:
            intrinsics->setParameter(ExtendedUnifiedCameraModel::Alpha, 0.5);
            intrinsics->setParameter(ExtendedUnifiedCameraModel::Beta, 1.);
            break;
        case CameraIntrinsicsType::Omnidirectional: {
            intrinsics->setParameter(OmnidirectionalCameraModel::Xi, 1.6);
        } break;
        // TODO: Give better initial values if possible
        case CameraIntrinsicsType::Pinhole:
        case CameraIntrinsicsType::PinholeRadialTangential:
        case CameraIntrinsicsType::Fisheye:
        case CameraIntrinsicsType::Fov:
        case CameraIntrinsicsType::Orthographic:
            break;
        default:
            break;
    }

    return viewId;
}

void CameraIntrinsicsCalibration::Impl::removeViewsByReprojectionError(
    double maxRPE, CameraId camId)
{
    std::map<ViewId, double> viewIdToRPE;
    for (const auto& viewId : m_scene->sharedCameraViewIds(camId)) {
        const double viewRPE =
            m_scene->calcViewReprojectionError(viewId).value();
        if (viewRPE > maxRPE) {
            viewIdToRPE.insert({viewId, viewRPE});
        }
    }

    for (const auto& [viewId, rpe] : viewIdToRPE) {
        m_scene->removeView(viewId);
        LOG(INFO) << "Remove view " << viewId << " (" << rpe << " > max "
                  << maxRPE << ")";
    }
}

void CameraIntrinsicsCalibration::Impl::removeViewsByGeometry(CameraId camId)
{
    std::vector<ViewId> viewIds;
    for (const auto& viewId : m_scene->sharedCameraViewIds(camId)) {
        const auto position = m_scene->view(viewId)->position();

        // NOTE: Optimization may give us views behind the calibration target
        if (position.z() < 0.) {
            viewIds.push_back(viewId);
        }
    }

    for (const auto& viewId : viewIds) {
        if (m_scene->removeView(viewId)) {
            LOG(INFO) << "Remove view " << viewId << " (Invalid geometry)";
        }
    }
}

///------- CameraIntrinsicsCalibration starts from here
bool CameraIntrinsicsCalibration::Options::isValid() const
{
    return sampleDistance > 0. && imageWidth > 0 && imageHeight > 0;
}

CameraIntrinsicsCalibration::CameraIntrinsicsCalibration(const Options& options)
    : d(std::make_unique<Impl>())
{
    d->init(options);
}

CameraIntrinsicsCalibration::~CameraIntrinsicsCalibration() = default;

void CameraIntrinsicsCalibration::setOptions(const Options& opts)
{
    d->m_opts = opts;
    d->updateConfigs();
}

const CameraIntrinsicsCalibration::Options&
CameraIntrinsicsCalibration::options() const
{
    return d->m_opts;
}

void CameraIntrinsicsCalibration::setupScene(
    std::shared_ptr<CalibBoardBase> board)
{
    mvs::setupBoardInScene(d->m_scene, *board.get());
}

void CameraIntrinsicsCalibration::addDetection(
    const StampedTargetDetection& detection, CameraId camId)
{
    if (!d->m_opts.isValid()) {
        LOG(ERROR) << "Failed to add detection: "
                      "Invalid configurations.";
        return;
    }

    d->initializeView(detection, camId);
}

void CameraIntrinsicsCalibration::clearAllViews(CameraId camId)
{
    d->m_scene->removeViews(camId);

    const auto found = d->m_existedViewPoses.find(camId);
    if (found != d->m_existedViewPoses.cend()) {
        d->m_existedViewPoses.erase(found);
    }
}

CameraIntrinsicsCalibration::Summary CameraIntrinsicsCalibration::calibrate(
    const std::vector<StampedTargetDetection>& detections, CameraId camId)
{
    using ErrorCode = CameraIntrinsicsCalibration::Summary::ErrorCode;

    Summary summary;
    if (!d->m_opts.isValid()) {
        LOG(ERROR) << "Failed to calibrate: "
                      "Invalid configurations.";
        summary.errCode = ErrorCode::InvalidConfigs;
        return summary;
    }

    d->initializeViews(detections, camId);

    return calibrate(camId);
}

CameraIntrinsicsCalibration::Summary CameraIntrinsicsCalibration::calibrate(
    CameraId camId)
{
    using ErrorCode = CameraIntrinsicsCalibration::Summary::ErrorCode;

    Summary summary;
    if (!d->m_opts.isValid()) {
        LOG(ERROR) << "Failed to calibrate: "
                      "Invalid configurations.";
        summary.errCode = ErrorCode::InvalidConfigs;
        return summary;
    }

    if (d->m_opts.checkDetectionCoverage) {
        if (!d->m_sampleStats.satisfy()) {
            LOG(ERROR) << "Failed to calibrate: "
                          "Poor observation converage.";
            summary.errCode = ErrorCode::PoorObservationCoverage;
            return summary;
        }
    }

    LOG(INFO) << "Total " << d->m_scene->viewCount(camId) << " views (Camera "
              << camId << ") are initialized.";

    const auto baError = d->startBA(camId);
    if (baError != ErrorCode::Success) {
        LOG(ERROR) << "Camera calibration BA failed.";
        summary.errCode = baError;
        return summary;
    }

    double avgViewRPE{0.};
    for (const auto& viewId : d->m_scene->sharedCameraViewIds(camId)) {
        const double viewRPE =
            d->m_scene->calcViewReprojectionError(viewId).value();
        avgViewRPE += viewRPE;
        VLOG(-1) << "View " << viewId << " RPE: " << viewRPE;
    }
    avgViewRPE /= d->m_scene->viewCount(camId);
    LOG(INFO) << "Average RMS reprojection error: " << avgViewRPE << " from "
              << d->m_scene->viewCount(camId) << " views.";

    summary.errCode = ErrorCode::Success;
    summary.finalRPE = avgViewRPE;

    return summary;
}

const Camera* CameraIntrinsicsCalibration::camera(CameraId camId) const
{
    const auto shareCamViewIds = d->m_scene->sharedCameraViewIds(camId);
    if (shareCamViewIds.empty()) {
        return nullptr;
    }

    return &d->m_scene->view(*shareCamViewIds.begin())->camera();
}

Scene::Ptr CameraIntrinsicsCalibration::scene() const { return d->m_scene; }

namespace key {
constexpr char kScope[]{"camera_intrinsics_calibration"};
constexpr char kVoxelGridSize[]{"voxel_grid_size"};
constexpr char kImageWidth[]{"image_width"};
constexpr char kImageHeight[]{"image_height"};
constexpr char kMinViewCount[]{"min_view_count"};
constexpr char kIntrinsicsType[]{"intrinsics_type"};
constexpr char kOptimizeBoardPoints[]{"optimize_board_points"};
} // namespace key

bool CameraIntrinsicsCalibration::setFromJson(const std::string& json)
{
    const auto j = nlohmann::json::parse(json);

    Options opts;
    opts.sampleDistance = j[key::kVoxelGridSize];
    opts.imageWidth = j[key::kImageWidth];
    opts.imageHeight = j[key::kImageHeight];
    opts.minViewCount = j[key::kMinViewCount];
    opts.intrinsicsType = magic_enum::enum_cast<CameraIntrinsicsType>(
                              j[key::kIntrinsicsType].get<std::string>())
                              .value_or(CameraIntrinsicsType::Pinhole);
    opts.optimizeBoardPoints = j[key::kOptimizeBoardPoints];

    setOptions(opts);

    return true;
}

void CameraIntrinsicsCalibration::toJsonString(std::string& json) const
{
    nlohmann::json j;
    j[key::kVoxelGridSize] = d->m_opts.sampleDistance;
    j[key::kImageWidth] = d->m_opts.imageWidth;
    j[key::kImageHeight] = d->m_opts.imageHeight;
    j[key::kMinViewCount] = d->m_opts.minViewCount;
    j[key::kIntrinsicsType] = magic_enum::enum_name(d->m_opts.intrinsicsType);
    j[key::kOptimizeBoardPoints] = d->m_opts.optimizeBoardPoints;

    json = j.dump();
}

} // namespace tl
