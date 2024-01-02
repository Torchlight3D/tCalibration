﻿#include "stereo_camera_calibration.h"

#include <ceres/ceres.h>
#include <Eigen/Core>

#include <AxCamera/DivisionUndistortionCameraModel>
#include <AxCamera/DoubleSphereCameraModel>
#include <AxCamera/ExtendedUnifiedCameraModel>
#include <AxCamera/FisheyeCameraModel>
#include <AxCamera/FovCameraModel>
#include <AxCamera/OmnidirectionalCameraModel>
#include <AxCamera/OrthographicCameraModel>
#include <AxCamera/PinholeCameraModel>
#include <AxCamera/PinholeRadialTangentialCameraModel>
#include <AxCore/ContainerUtils>
#include <AxMath/EigenUtils>
#include <AxMath/MathBase>

namespace thoht {

using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

template <class CameraIntrinsic>
struct StereoReprojectionError
{
    const Vector3d m_objPoint;
    const Vector2d m_leftImgPoint;
    const Vector2d m_rightImgPoint;

    StereoReprojectionError(const Vector3d& objPoint,
                            const Vector2d& leftImgPoint,
                            const Vector2d& rightImgPoint)
        : m_objPoint(objPoint),
          m_leftImgPoint(leftImgPoint),
          m_rightImgPoint(rightImgPoint)
    {
    }

    template <typename T>
    bool operator()(const T* const leftIntrinsics,
                    const T* const rightIntrinsics, const T* const q_l,
                    const T* const t_l, const T* const q_l_r,
                    const T* const t_l_r, T* residuals) const
    {
        auto transformPoint =
            [](const Eigen::Vector3<T>& point, const Eigen::Quaternion<T>& q,
               const Eigen::Vector3<T>& t) -> Eigen::Vector3<T> {
            Eigen::Vector3<T> aa;
            T q_ceres[4] = {q.w(), q.x(), q.y(), q.z()};
            ceres::QuaternionToAngleAxis(q_ceres, aa.data());

            const Eigen::Vector3<T> adjusted_point = point - t;
            Eigen::Vector3<T> result;
            ceres::AngleAxisRotatePoint(aa.data(), adjusted_point.data(),
                                        result.data());

            return result;
        };

        // Input, TODO: use Eigen::Map
        const Eigen::Vector3<T> objPoint = m_objPoint.cast<T>();
        const Eigen::Vector3<T> tvec_l{t_l};
        const Eigen::Quaternion<T> quat_l{q_l};
        const Eigen::Vector3<T> tvec_l_r{t_l_r};
        const Eigen::Quaternion<T> quat_l_r{q_l_r};

        // Left
        const auto objPointInLeft = transformPoint(objPoint, quat_l, tvec_l);

        Eigen::Vector2<T> projLeftPoint;
        CameraIntrinsic::spaceToPixel(leftIntrinsics, objPointInLeft.data(),
                                      projLeftPoint.data());

        // Right
        const Eigen::Quaternion<T> quat_r = quat_l_r * quat_l;
        const Eigen::Vector3<T> tvec_r =
            -quat_r.toRotationMatrix().transpose() * tvec_l_r + tvec_l;

        const auto objPointInRight = transformPoint(objPoint, quat_r, tvec_r);

        Eigen::Vector2<T> projRightPoint;
        CameraIntrinsic::spaceToPixel(rightIntrinsics, objPointInRight.data(),
                                      projRightPoint.data());

        residuals[0] = projLeftPoint(0) - T(m_leftImgPoint(0));
        residuals[1] = projLeftPoint(1) - T(m_leftImgPoint(1));
        residuals[2] = projRightPoint(0) - T(m_rightImgPoint(0));
        residuals[3] = projRightPoint(1) - T(m_rightImgPoint(1));

        return true;
    }

    // FIXME: Not used yet, waiting for fix...
    template <class CameraIntrinsics>
    static ceres::CostFunction* create(
        const Vector3d& objPoint, const Vector2d& leftImgPoint,
        const Vector2d& rightImgPoint,
        typename CameraIntrinsics::Ptr intrinsics)
    {
        constexpr int kResidualSize{4}; // 2 from left, 2 from right
        constexpr int kQuaternionSize{4};
        constexpr int kTranslationSize{3};
        return new ceres::AutoDiffCostFunction<
            StereoReprojectionError<CameraIntrinsic>, kResidualSize,
            CameraIntrinsics::IntrinsicsSize, CameraIntrinsics::IntrinsicsSize,
            kQuaternionSize, kTranslationSize, kQuaternionSize,
            kTranslationSize>(new StereoReprojectionError<CameraIntrinsics>(
            objPoint, leftImgPoint, rightImgPoint));
    }
};

// FIXME: STUPID!!!!!
ceres::CostFunction* createStereoReprojectionError(
    const Vector3d& objPoint, const Vector2d& leftImgPoint,
    const Vector2d& rightImgPoint, CameraIntrinsics::Ptr intrinsics)
{
    constexpr int kResidualSize{4}; // 2 from left, 2 from right
    constexpr int kQuaternionSize{4};
    constexpr int kTranslationSize{3};

    switch (intrinsics->type()) {
        case CameraIntrinsics::Type::Pinhole:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<PinholeCameraModel>, kResidualSize,
                PinholeCameraModel::IntrinsicsSize,
                PinholeCameraModel::IntrinsicsSize, kQuaternionSize,
                kTranslationSize, kQuaternionSize, kTranslationSize>(
                new StereoReprojectionError<PinholeCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::PinholeRadialTangential:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<PinholeRadialTangentialCameraModel>,
                kResidualSize,
                PinholeRadialTangentialCameraModel::IntrinsicsSize,
                PinholeRadialTangentialCameraModel::IntrinsicsSize,
                kQuaternionSize, kTranslationSize, kQuaternionSize,
                kTranslationSize>(
                new StereoReprojectionError<PinholeRadialTangentialCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::Fisheye:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<FisheyeCameraModel>, kResidualSize,
                FisheyeCameraModel::IntrinsicsSize,
                FisheyeCameraModel::IntrinsicsSize, kQuaternionSize,
                kTranslationSize, kQuaternionSize, kTranslationSize>(
                new StereoReprojectionError<FisheyeCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::Fov:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<FOVCameraModel>, kResidualSize,
                FOVCameraModel::IntrinsicsSize, FOVCameraModel::IntrinsicsSize,
                kQuaternionSize, kTranslationSize, kQuaternionSize,
                kTranslationSize>(new StereoReprojectionError<FOVCameraModel>(
                objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::DivisionUndistortion:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<DivisionUndistortionCameraModel>,
                kResidualSize, DivisionUndistortionCameraModel::IntrinsicsSize,
                DivisionUndistortionCameraModel::IntrinsicsSize,
                kQuaternionSize, kTranslationSize, kQuaternionSize,
                kTranslationSize>(
                new StereoReprojectionError<DivisionUndistortionCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::DoubleSphere:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<DoubleSphereCameraModel>, kResidualSize,
                DoubleSphereCameraModel::IntrinsicsSize,
                DoubleSphereCameraModel::IntrinsicsSize, kQuaternionSize,
                kTranslationSize, kQuaternionSize, kTranslationSize>(
                new StereoReprojectionError<DoubleSphereCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::ExtendedUnified:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<ExtendedUnifiedCameraModel>,
                kResidualSize, ExtendedUnifiedCameraModel::IntrinsicsSize,
                ExtendedUnifiedCameraModel::IntrinsicsSize, kQuaternionSize,
                kTranslationSize, kQuaternionSize, kTranslationSize>(
                new StereoReprojectionError<ExtendedUnifiedCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::Omnidirectional:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<OmnidirectionalCameraModel>,
                kResidualSize, OmnidirectionalCameraModel::IntrinsicsSize,
                OmnidirectionalCameraModel::IntrinsicsSize, kQuaternionSize,
                kTranslationSize, kQuaternionSize, kTranslationSize>(
                new StereoReprojectionError<OmnidirectionalCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));
        case CameraIntrinsics::Type::Orthographic:
            return new ceres::AutoDiffCostFunction<
                StereoReprojectionError<OrthographicCameraModel>, kResidualSize,
                OrthographicCameraModel::IntrinsicsSize,
                OrthographicCameraModel::IntrinsicsSize, kQuaternionSize,
                kTranslationSize, kQuaternionSize, kTranslationSize>(
                new StereoReprojectionError<OrthographicCameraModel>(
                    objPoint, leftImgPoint, rightImgPoint));

        default:
            break;
    }

    return nullptr;
}

///------- StereoCameraCalibration::Impl starts from here
class StereoCameraCalibration::Impl
{
public:
    Impl();

    void init(const Options& options);

    // Loop all the stereo pairs, and find the pair with smallest rpe
    void findInitialPose(const std::vector<ViewId>& leftViewIds,
                         const std::vector<ViewId>& rightViewIds,
                         Eigen::Quaterniond& bestOrientation,
                         Eigen::Vector3d& bestTranslation) const;

    void testTransform(const std::vector<ViewId>& leftViewIds,
                       const std::vector<ViewId>& rightViewIds,
                       const Eigen::Quaterniond& bestOrientation,
                       const Eigen::Vector3d& bestTranslation) const;

    // Use with calibrated scene interface
    bool checkScene(const Scene::ConstPtr scene) const;

public:
    Options m_opts;
    Scene::Ptr m_scene{nullptr}; // not own
    Quaterniond m_orientation;
    Vector3d m_translation;
};

StereoCameraCalibration::Impl::Impl() {}

void StereoCameraCalibration::Impl::init(const Options& options)
{
    m_opts = options;
}

void StereoCameraCalibration::Impl::findInitialPose(
    const std::vector<ViewId>& leftViewIds,
    const std::vector<ViewId>& rightViewIds, Quaterniond& orientation,
    Vector3d& translation) const
{
    const auto pairCnt = leftViewIds.size();

    double minRPE = math::kMaxDouble;
    for (size_t i{0}; i < pairCnt; ++i) {
        const auto& thisLeftCamera = m_scene->view(leftViewIds[i])->camera();
        const auto& thisRightCamera = m_scene->view(rightViewIds[i])->camera();

        const Quaterniond q_l{thisLeftCamera.orientationAsRotationMatrix()};
        const auto t_l = thisLeftCamera.position();
        const Quaterniond q_r{thisRightCamera.orientationAsRotationMatrix()};
        const auto t_r = thisRightCamera.position();

        const Quaterniond q_l_r = q_r * q_l.conjugate();
        const Vector3d t_l_r = q_r.toRotationMatrix() * (t_l - t_r);

        Vector3dList rvecs_r;
        Vector3dList tvecs_r;
        rvecs_r.reserve(pairCnt);
        tvecs_r.reserve(pairCnt);
        for (const auto& leftViewId : leftViewIds) {
            const auto& leftCamera = m_scene->view(leftViewId)->camera();

            const Quaterniond q_l{leftCamera.orientationAsRotationMatrix()};
            const auto t_l = leftCamera.position();

            const Quaterniond q_r = q_l_r * q_l;
            const Vector3d t_r =
                -q_l_r.toRotationMatrix().transpose() * t_l_r + t_l;

            Vector3d rvec_r;
            math::QuaternionToAngleAxis(q_r, rvec_r);

            rvecs_r.push_back(rvec_r);
            tvecs_r.push_back(t_r);
        }

        double rpe{0.};
        size_t pointCnt{0};
        for (size_t idx{0}; idx < rightViewIds.size(); ++idx) {
            const auto& rightViewId = rightViewIds[idx];
            const auto trackCnt = m_scene->view(rightViewId)->trackCount();
            const double avgRPE = m_scene->calcViewReprojectionError(
                rightViewId, false, &(rvecs_r[idx]), &(tvecs_r[idx]));
            pointCnt += trackCnt;
            rpe += avgRPE;
        }
        rpe /= pointCnt;

        if (rpe < minRPE) {
            minRPE = rpe;
            orientation = q_l_r;
            translation = t_l_r;
        }
    }
}

void StereoCameraCalibration::Impl::testTransform(
    const std::vector<ViewId>& leftViewIds,
    const std::vector<ViewId>& rightViewIds, const Quaterniond& q_l_r,
    const Vector3d& t_l_r) const
{
    Vector3dList rvecs_l, tvecs_l;
    Vector3dList rvecs_r, tvecs_r;
    for (size_t i{0}; i < leftViewIds.size(); ++i) {
        const auto& leftCamera = m_scene->view(leftViewIds[i])->camera();
        const Vector3d rvec_l = leftCamera.orientationAsAngleAxis();
        const auto q_l = math::AngleAxisToQuaternion(rvec_l);
        const auto tvec_l = leftCamera.position();
        rvecs_l.push_back(rvec_l);
        tvecs_l.push_back(tvec_l);

        const Quaterniond q_r = q_l_r * q_l;
        const Vector3d t_r = q_l_r.toRotationMatrix() * tvec_l + t_l_r;

        Vector3d rvec_r;
        math::QuaternionToAngleAxis(q_r, rvec_r);
        rvecs_r.push_back(rvec_r);
        tvecs_r.push_back(t_r);
    }

    auto calcRPE = [this](const std::vector<ViewId>& viewIds,
                          const Vector3dList& rvecs, const Vector3dList& tvecs,
                          const std::string& hint) {
        double rpe{0.};
        size_t pointCnt{0};
        for (size_t i{0}; i < viewIds.size(); ++i) {
            const auto& viewId = viewIds[i];
            const auto trackCnt = m_scene->view(viewId)->trackCount();
            const double avgRPE = m_scene->calcViewReprojectionError(
                viewId, false, &(rvecs[i]), &(tvecs[i]));
            pointCnt += trackCnt;
            rpe += (avgRPE * trackCnt);
        }
        rpe /= pointCnt;
    };

    calcRPE(leftViewIds, rvecs_l, tvecs_l, "Left");
    calcRPE(rightViewIds, rvecs_r, tvecs_r, "Right");
}

bool StereoCameraCalibration::Impl::checkScene(
    const Scene::ConstPtr scene) const
{
    if (!scene) {
        LOG(WARNING) << "Invalid scene: "
                        "Scene is null.";
        return false;
    }

    const auto cameraIds = scene->cameraIds();
    if (cameraIds.size() < 2) {
        LOG(WARNING) << "Invalid scene: "
                        "Required at least two cameras in the scene for stereo "
                        "calibration.";
        return false;
    }

    const bool calibrated = std::all_of(
        cameraIds.cbegin(), cameraIds.cend(), [&scene](const auto& camId) {
            return scene->camera(camId)->calibrated();
        });
    if (!calibrated) {
        LOG(WARNING) << "Invalid scene: "
                        "Not all the cameras in scene are calibrated.";
        return false;
    }

    return true;
}

///------- StereoCameraCalibration starts from here
StereoCameraCalibration::StereoCameraCalibration(const Options& options)
    : d(std::make_unique<Impl>())
{
    d->init(options);
}

StereoCameraCalibration::~StereoCameraCalibration() = default;

const StereoCameraCalibration::Options& StereoCameraCalibration::options() const
{
    return d->m_opts;
}

StereoCameraCalibration::Options& StereoCameraCalibration::rOptions()
{
    return d->m_opts;
}

void StereoCameraCalibration::setScene(const Scene::Ptr scene)
{
    if (!d->checkScene(scene)) {
        return;
    }

    d->m_scene = scene;
}

void StereoCameraCalibration::setupScene(const CalibBoardBase& board)
{
    // TODO:
}

bool StereoCameraCalibration::calibrate()
{
    if (!d->checkScene(d->m_scene)) {
        return false;
    }

    LOG(INFO) << "======= "
                 "Start stereo calibration."
                 " =======";
    double stereoAvgViewRPE{0.};
    // NOTE: Now the first loop is useless.
    const auto camIds = d->m_scene->cameraIds();
    for (const auto& leftCamId : camIds) {
        const auto rightCamId = d->m_scene->pairCameraIdOf(leftCamId);
        if (rightCamId == kInvalidCameraId) {
            continue;
        }

        // 1. Extract valid view pairs
        std::vector<ViewId> leftViewIds;
        std::vector<ViewId> rightViewIds;
        const auto& timeToViewIds = d->m_scene->timeToViewIds();
        for (const auto& [_, viewIds] : timeToViewIds) {
            // Now only support 1 pair of stereo view
            if (viewIds.size() != 2) {
                continue;
            }

            // Assume views were added from left to right
            const auto leftViewId = *viewIds.begin();
            const auto rightViewId = *std::next(viewIds.begin());
            const auto& leftCamera = d->m_scene->view(leftViewId)->camera();
            const auto& rightCamera = d->m_scene->view(rightViewId)->camera();
            if (leftCamera.cameraIntrinsicsModel() !=
                rightCamera.cameraIntrinsicsModel()) {
                continue;
            }

            leftViewIds.push_back(leftViewId);
            rightViewIds.push_back(rightViewId);
        }

        QuaterniondList quats_l;
        Vector3dList tvecs_l;
        for (size_t i{0}; i < leftViewIds.size(); ++i) {
            const auto& leftCamera = d->m_scene->view(leftViewIds[i])->camera();
            const Quaterniond q_l{leftCamera.orientationAsRotationMatrix()};
            const auto tvec_l = leftCamera.position();
            quats_l.push_back(q_l);
            tvecs_l.push_back(tvec_l);
        }

        // 2. Find best initial left-to-right transform
        Quaterniond q_l_r;
        Vector3d t_l_r;
        d->findInitialPose(leftViewIds, rightViewIds, q_l_r, t_l_r);

        LOG(INFO) << "Initial transform: " << t_l_r.transpose() << "; "
                  << q_l_r.coeffs().transpose();

        // 3. Test best initial transform to all current transforms
        //        d->testTransform(leftViewIds, rightViewIds, q_l_r, t_l_r);

        // 4. Optimize poses
        ceres::Problem problem;

        // WARNING:
        // 1. If use non-const interface to access intrinsics, the intrinsics
        // would change in some cases (???)
        // 2. If use temporary variable inside the loop, it would cause
        // segmentation fault, or failed ceres optimization (???).
        const auto& leftCamera = d->m_scene->view(leftViewIds[0])->camera();
        const auto& rightCamera = d->m_scene->view(rightViewIds[0])->camera();
        auto leftParams = leftCamera.parameters();
        auto rightParams = rightCamera.parameters();
        for (size_t i{0}; i < leftViewIds.size(); ++i) {
            const auto* leftView = d->m_scene->view(leftViewIds[i]);
            const auto* rightView = d->m_scene->view(rightViewIds[i]);

            // Add tracks that are observed by both views
            auto leftTrackIds = leftView->trackIds();
            auto rightTrackIds = rightView->trackIds();
            std::sort(leftTrackIds.begin(), leftTrackIds.end());
            std::sort(rightTrackIds.begin(), rightTrackIds.end());

            std::vector<TrackId> sharedTrackIds;
            std::set_intersection(leftTrackIds.begin(), leftTrackIds.end(),
                                  rightTrackIds.begin(), rightTrackIds.end(),
                                  std::back_inserter(sharedTrackIds));

            for (const auto& trackId : sharedTrackIds) {
                const Vector3d objPoint =
                    d->m_scene->track(trackId)->position().head<3>();
                const auto& leftImgPoint = leftView->featureOf(trackId)->point_;
                const auto& rightImgPoint =
                    rightView->featureOf(trackId)->point_;

                auto* costFunc = createStereoReprojectionError(
                    objPoint, leftImgPoint, rightImgPoint,
                    leftCamera.cameraIntrinsics());
                auto* lossFunc = new ceres::CauchyLoss(1.0);
                problem.AddResidualBlock(
                    costFunc, lossFunc,
                    d->m_scene->rView(leftViewIds[0])->rCamera().rIntrinsics(),
                    d->m_scene->rView(rightViewIds[0])->rCamera().rIntrinsics(),
                    quats_l[i].coeffs().data(), tvecs_l[i].data(),
                    q_l_r.coeffs().data(), t_l_r.data());
            }
        }

        for (size_t i{0}; i < leftViewIds.size(); ++i) {
            problem.SetManifold(quats_l[i].coeffs().data(),
                                new ceres::EigenQuaternionManifold);
        }

        problem.SetManifold(q_l_r.coeffs().data(),
                            new ceres::EigenQuaternionManifold);

        ceres::Solver::Options options;
        options.max_num_iterations = 1000;
        options.num_threads = 8;
        options.minimizer_progress_to_stdout = false;

        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.function_tolerance = 1e-3;
        options.gradient_tolerance = 1e-3;
        options.parameter_tolerance = 1e-3;
        options.max_trust_region_radius = 1e4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        LOG(INFO) << summary.FullReport();

        d->m_orientation = q_l_r;
        d->m_translation = t_l_r;

        double avgViewRPE{0.};
        for (const auto& viewId : d->m_scene->sharedCameraViewIds(leftCamId)) {
            const double viewRPE =
                d->m_scene->calcViewReprojectionError(viewId);
            avgViewRPE += viewRPE;
            // VLOG(-1) << "View " << viewId << " RPE: " << viewRPE;
        }
        avgViewRPE /= d->m_scene->viewCount(leftCamId);
        stereoAvgViewRPE += avgViewRPE;
        continue;

        // 5. Set the optimized transform back to scene
        d->testTransform(leftViewIds, rightViewIds, q_l_r, t_l_r);
        for (size_t i{0}; i < leftViewIds.size(); ++i) {
            const auto& q_l = quats_l[i];
            const auto& t_l = tvecs_l[i];

            auto& leftCamera = d->m_scene->rView(leftViewIds[i])->rCamera();
            leftCamera.setOrientationFromRotationMatrix(q_l.toRotationMatrix());
            leftCamera.setPosition(t_l);

            const Quaterniond q_r = q_l_r * q_l;
            const Vector3d t_r = q_l_r.toRotationMatrix() * t_l + t_l_r;

            auto& rightCamera = d->m_scene->rView(rightViewIds[i])->rCamera();
            rightCamera.setOrientationFromRotationMatrix(
                q_r.toRotationMatrix());
            rightCamera.setPosition(t_r);
        }
    }
    LOG(INFO) << "Stereo camera average reprojection error: "
              << stereoAvgViewRPE / 2;
    return true;
}

bool StereoCameraCalibration::calibrate(const StampedTargetDetections& left,
                                        const StampedTargetDetections& right)
{
    // TODO:
    return false;
}

void StereoCameraCalibration::getTransform(Quaterniond& orientation,
                                           Vector3d& translation) const
{
    orientation = d->m_orientation;
    translation = d->m_translation;
}

Scene::Ptr StereoCameraCalibration::scene() const { return d->m_scene; }

namespace key {

}

bool StereoCameraCalibration::setFromJson(const std::string& json)
{
    // TODO:
    return false;
}

void StereoCameraCalibration::toJsonString(std::string& json) const
{
    // TODO:
}

} // namespace thoht
