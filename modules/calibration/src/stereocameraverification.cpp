#include "stereocameraverification.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp> // For calib file parsing
#include <opencv2/imgproc.hpp>

#include <tCamera/coc/CataCamera>
#include <tCamera/coc/EquidistantCamera>
#include <tCamera/coc/PinholeCamera>
#include <tCamera/coc/PinholeFullCamera>
#include <tCamera/coc/CameraFactory>
#include <tCore/ContainerUtils>
#include <tCore/Math>

namespace tl {

using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

using Matrix34d = Eigen::Matrix<double, 3, 4>;

namespace {

template <typename T>
std::vector<double> calcPointPairsDistance(
    const std::vector<cv::Point3_<T>> &points)
{
    if (points.size() < 2) {
        return {};
    }

    const auto len = points.size();

    std::vector<double> distances;
    distances.reserve(math::combination(len, size_t{2}));
    for (size_t i{0}; i < len - 1; ++i) {
        for (size_t j{i + 1}; j < len; ++j) {
            distances.emplace_back(cv::norm(points[i] - points[j]));
        }
    }

    return distances;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d hat;
    hat << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
    return hat;
}

double reprojectionError(const Eigen::Matrix3d &Ri, const Eigen::Vector3d &Pi,
                         const Eigen::Matrix3d &rici,
                         const Eigen::Vector3d &tici, const Eigen::Matrix3d &Rj,
                         const Eigen::Vector3d &Pj, const Eigen::Matrix3d &ricj,
                         const Eigen::Vector3d &ticj, double depth,
                         const Eigen::Vector3d &uvi, const Eigen::Vector3d &uvj)
{
    Eigen::Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Eigen::Vector3d pts_cj =
        ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void triangulatePoint(const Matrix34d &pose0, const Matrix34d &pose1,
                      const Eigen::Vector2d &point0,
                      const Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * pose0.row(2) - pose0.row(0);
    design_matrix.row(1) = point0[1] * pose0.row(2) - pose0.row(1);
    design_matrix.row(2) = point1[0] * pose1.row(2) - pose1.row(0);
    design_matrix.row(3) = point1[1] * pose1.row(2) - pose1.row(1);
    Eigen::Vector4d triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

} // namespace

///------- StereoReprojectionError starts from here
// TODO: Maybe duplicated
struct StereoReprojectionError
{
    const Vector3d m_objPoint;
    const Vector2d m_leftImgPoint;
    const Vector2d m_rightImgPoint;
    const Vector3d m_tvec_r_l;
    const Quaterniond m_quat_r_l;
    const double m_focal_length;

    StereoReprojectionError(const Vector3d &objPoint,
                            const Vector2d &leftImgPoint,
                            const Vector2d &rightImgPoint,
                            const Quaterniond &quat_r_l,
                            const Vector3d &tvec_r_l, double focal_length)
        : m_objPoint(objPoint),
          m_leftImgPoint(leftImgPoint),
          m_rightImgPoint(rightImgPoint),
          m_tvec_r_l(tvec_r_l),
          m_quat_r_l(quat_r_l),
          m_focal_length(focal_length)
    {
    }

    template <typename T>
    bool operator()(const T *const q_l, const T *const t_l, T *residuals) const
    {
        auto transformPoint =
            [](const Eigen::Vector3<T> &point, const Eigen::Quaternion<T> &q,
               const Eigen::Vector3<T> &t) -> Eigen::Vector3<T> {
            Eigen::Matrix<T, 3, 3, Eigen::RowMajor> aa;
            T q_ceres[4] = {q.w(), q.x(), q.y(), q.z()};
            ceres::QuaternionToRotation(q_ceres, aa.data());

            const Eigen::Vector3<T> result = aa * point + t;

            return result;
        };

        // Input, TODO: use Eigen::Map
        const Eigen::Vector3<T> objPoint = m_objPoint.cast<T>();
        const Eigen::Vector3<T> tvec_l{t_l};
        const Eigen::Quaternion<T> quat_l{q_l};
        const Eigen::Vector3<T> tvec_r_l = m_tvec_r_l.cast<T>();    // l to r
        const Eigen::Quaternion<T> quat_r_l = m_quat_r_l.cast<T>(); // l to r

        // Left
        const auto objPointInLeft = transformPoint(objPoint, quat_l, tvec_l);

        Eigen::Vector2<T> projLeftPoint;
        projLeftPoint.x() = objPointInLeft.x() / objPointInLeft.z();
        projLeftPoint.y() = objPointInLeft.y() / objPointInLeft.z();

        // Right
        const auto objPointInRight =
            transformPoint(objPointInLeft, quat_r_l, tvec_r_l);

        Eigen::Vector2<T> projRightPoint;
        projRightPoint.x() = objPointInRight.x() / objPointInRight.z();
        projRightPoint.y() = objPointInRight.y() / objPointInRight.z();

        residuals[0] = projLeftPoint(0) - T(m_leftImgPoint(0));
        residuals[1] = projLeftPoint(1) - T(m_leftImgPoint(1));
        residuals[2] = projRightPoint(0) - T(m_rightImgPoint(0));
        residuals[3] = projRightPoint(1) - T(m_rightImgPoint(1));

        residuals[0] *= T(m_focal_length);
        residuals[1] *= T(m_focal_length);
        residuals[2] *= T(m_focal_length);
        residuals[3] *= T(m_focal_length);
        return true;
    }
};

ceres::CostFunction *createStereoReprojectionError1(
    const Vector3d &objPoint, const Vector2d &leftImgPoint,
    const Vector2d &rightImgPoint, const Quaterniond &quat_r_l,
    const Vector3d &tvec_r_l, double focal_length)
{
    constexpr int kResidualSize{4}; // 2 from left, 2 from right
    constexpr int kQuaternionSize{4};
    constexpr int kTranslationSize{3};

    return new ceres::AutoDiffCostFunction<StereoReprojectionError,
                                           kResidualSize, kQuaternionSize,
                                           kTranslationSize>(
        new StereoReprojectionError(objPoint, leftImgPoint, rightImgPoint,
                                    quat_r_l, tvec_r_l, focal_length));
}

///------- StereoCameraVerification::VerifyData starts from here
namespace key {
constexpr char kCameras[]{"cameras"};
constexpr char kUuid[]{"uuid"};
constexpr char kFocalLength[]{"focal_length"};
} // namespace key

namespace internal {

bool loadDataFromNode(const cv::FileNode &node,
                      StereoCameraVerification::VerifyData &data,
                      std::string &uuid)
{
    // Dont care if uuid exists or not
    const auto uuidNode = node[key::kUuid];
    if (!uuidNode.empty()) {
        uuid = node[key::kUuid].string();
    }

    const auto focalLengthNode = node[key::kFocalLength];
    if (!focalLengthNode.empty()) {
        data.focal_length = focalLengthNode;
    }
    else {
        constexpr double kMagicFocalLength = 328.;
        data.focal_length = kMagicFocalLength;
    }

    const auto camerasNode = node[key::kCameras];
    if (camerasNode.size() != static_cast<size_t>(StereoSize)) {
        LOG(ERROR) << "Failed to read parameters file: "
                      "Only stereo camera is supported.";
        return false;
    }

    auto toCameraKey = [](int index) -> std::string {
        return "cam" + std::to_string(index);
    };

    const auto leftNode = camerasNode[toCameraKey(CameraLeft)];
    const auto rightNode = camerasNode[toCameraKey(CameraRight)];
    if (leftNode[camodocal::Camera::Parameters::keyModelType].string() !=
        rightNode[camodocal::Camera::Parameters::keyModelType].string()) {
        LOG(ERROR)
            << "Failed to read parameters file: "
               "Only stereo camera with identical camera model is supported.";
        return false;
    }

    auto readCameraParameters = [](camodocal::Camera::Ptr &camera,
                                   const cv::FileNode &node) -> bool {
        camera = camodocal::CameraFactory::instance()->create(node);
        return camera.get();
    };

    auto &rLeftCam = data.cameras[CameraLeft];
    auto &rRightCam = data.cameras[CameraRight];
    const bool success = readCameraParameters(rLeftCam, leftNode) &&
                         readCameraParameters(rRightCam, rightNode);

    if (!success) {
        LOG(ERROR) << "Failed to read parameters file: "
                      "Not both camera nodes are correctly parsed.";
        return false;
    }

    const auto &leftCam = data.cameras[CameraLeft];
    const auto &rightCam = data.cameras[CameraRight];
    if (leftCam->imageSize() != rightCam->imageSize()) {
        LOG(ERROR)
            << "Failed to calculate stereo rectification: "
               "Only stereo camera with identical image size is supported.";
        return false;
    }

    auto toCamToImuTransformKey = [](int index) -> std::string {
        return "body_T_cam" + std::to_string(index);
    };

    std::array<cv::Mat, StereoSize> camToImuTransforms;
    node[toCamToImuTransformKey(CameraLeft)] >> camToImuTransforms[CameraLeft];
    node[toCamToImuTransformKey(CameraRight)] >>
        camToImuTransforms[CameraRight];

    const auto &T_i_l = camToImuTransforms[CameraLeft];
    const auto &T_i_r = camToImuTransforms[CameraRight];
    // read extrinsics between sensors

    {
        Eigen::Matrix4d T;
        cv::cv2eigen(T_i_l, T);
        data.ric[CameraLeft] = T.block<3, 3>(0, 0);
        data.tic[CameraLeft] = T.block<3, 1>(0, 3);
    }

    {
        Eigen::Matrix4d T;
        cv::cv2eigen(T_i_r, T);
        data.ric[CameraRight] = T.block<3, 3>(0, 0);
        data.tic[CameraRight] = T.block<3, 1>(0, 3);
    }

    data.rot_c1c0 = data.ric[CameraRight].transpose() * data.ric[CameraLeft];
    data.t_c1c0 = data.ric[CameraRight].transpose() *
                  (data.tic[CameraLeft] - data.tic[CameraRight]);
    Eigen::Matrix3d tc1c0_skew = skew(data.t_c1c0);
    data.essential_c1c0 = tc1c0_skew * data.rot_c1c0;

    return true;
}

} // namespace internal

bool StereoCameraVerification::VerifyData::loadFromFile(
    const std::string &filename, VerifyData &data, std::string &uuid)
{
    cv::FileStorage fs{filename, cv::FileStorage::READ};
    if (!fs.isOpened()) {
        LOG(ERROR) << "Failed to read paramters file: "
                      "Invalid filename. "
                   << filename;
        return false;
    }

    return internal::loadDataFromNode(fs.root(), data, uuid);
}

bool StereoCameraVerification::VerifyData::loadFromText(
    const std::string &bytes, VerifyData &data, std::string &uuid)
{
    cv::FileStorage fs{bytes, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    return internal::loadDataFromNode(fs.root(), data, uuid);
}

///------- StereoCameraVerification::Options starts from here
bool StereoCameraVerification::Options::isValid() const
{
    return !chessPatternSize.empty() && !chessPatternSize.empty();
}

///------- StereoCameraVerification::Impl starts from here
class StereoCameraVerification::Impl
{
public:
    explicit Impl(const VerifyData &data);

    void init(const Options &options);

    void updateConfigs();
    void updateBoardPoints();
    void clearDebugData();

    void reorderPoints(std::vector<cv::Point2f> &pts);
    void undistortPoints();

    double calcVioTrackerRejectRate();
    double calcVioEstimatorRejectRate();
    double solveStereoPose();
    double calcRelativeSizeErrMedian();
    double calcRelativePtsPosErrMedian();

public:
    Options opts_;
    const VerifyData &data_;

    int total_pts_num_;
    int total_pts_dis_num_;

    std::vector<cv::Point3d> chessboard_pos_;

    std::vector<cv::Point2f> cam0_pts_;
    std::vector<cv::Point2f> cam1_pts_;
    std::vector<cv::Point2d> cam0_unpts_;
    std::vector<cv::Point2d> cam1_unpts_;

    std::vector<cv::Point3d> triangulated_pts_in_cam0_;
    std::vector<cv::Point3d> chess_pts_in_cam0_by_opti_pose_;

    std::vector<double> real_chess_dis_;
    std::vector<double> triangulated_chess_dis_;

    // Cache for debug
    cv::Mat left_img_, right_img_;
    std::vector<uchar> status_track_;
    std::vector<uchar> status_esti1_;
    std::vector<uchar> status_esti2_;
    std::vector<uchar> status_esti3_;
};

StereoCameraVerification::Impl::Impl(const VerifyData &data) : data_(data) {}

void StereoCameraVerification::Impl::init(const Options &options)
{
    opts_ = options;
    updateConfigs();
}

void StereoCameraVerification::Impl::updateConfigs()
{
    updateBoardPoints();

    total_pts_num_ = opts_.chessPatternSize.area();
    total_pts_dis_num_ = math::combination(total_pts_num_, 2);
    real_chess_dis_.reserve(total_pts_dis_num_);
    triangulated_chess_dis_.reserve(total_pts_dis_num_);
}

void StereoCameraVerification::Impl::updateBoardPoints()
{
    chessboard_pos_.clear();
    for (int r{0}; r < opts_.chessPatternSize.height; r++) {
        for (int c{0}; c < opts_.chessPatternSize.width; c++) {
            chessboard_pos_.emplace_back(opts_.chessSize.width * c,
                                         opts_.chessSize.height * r, 0.);
        }
    }
}

void StereoCameraVerification::Impl::clearDebugData()
{
    left_img_.release();
    right_img_.release();
    status_track_.clear();
    status_esti1_.clear();
    status_esti2_.clear();
    status_esti3_.clear();
}

double StereoCameraVerification::Impl::calcRelativeSizeErrMedian()
{
    std::vector<double> errors;
    errors.reserve(real_chess_dis_.size());
    for (size_t i{0}; i < real_chess_dis_.size(); i++) {
        errors.emplace_back(
            fabs(triangulated_chess_dis_[i] / real_chess_dis_[i] - 1.));
    }

    return con::FindMedian(errors);
}

double StereoCameraVerification::Impl::calcRelativePtsPosErrMedian()
{
    std::vector<double> errors;
    for (size_t i{0}; i < triangulated_pts_in_cam0_.size(); i++) {
        const double error = cv::norm(triangulated_pts_in_cam0_[i] -
                                      chess_pts_in_cam0_by_opti_pose_[i]) /
                             cv::norm(chess_pts_in_cam0_by_opti_pose_[i]);
        errors.emplace_back(error);
    }

    return con::FindMedian(errors);
}

// QUEST: First point must be at tl, and last one must be at br. Why???
void StereoCameraVerification::Impl::reorderPoints(
    std::vector<cv::Point2f> &pts)
{
    const auto &first = pts.front();
    const auto &last = pts.back();
    if (first.x > last.x && first.y > last.y) {
        std::reverse(pts.begin(), pts.end());
        LOG(INFO) << "chessboard reversed";
    }
    else {
        LOG_IF(FATAL, (first.x - last.x) * (first.y - last.y) < 0.);
    }
}

void StereoCameraVerification::Impl::undistortPoints()
{
    auto n = cam0_pts_.size();
    cam0_unpts_.clear();
    cam1_unpts_.clear();

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d x0;

        data_.cameras[kCameraLeftId]->liftProjective(
            Eigen::Vector2d(cam0_pts_[i].x, cam0_pts_[i].y), x0);
        cam0_unpts_.emplace_back(x0(0) / x0(2), x0(1) / x0(2));

        data_.cameras[kCameraRightId]->liftProjective(
            Eigen::Vector2d(cam1_pts_[i].x, cam1_pts_[i].y), x0);
        cam1_unpts_.emplace_back(x0(0) / x0(2), x0(1) / x0(2));
    }
}

double StereoCameraVerification::Impl::calcVioTrackerRejectRate()
{
    // Reject by epipolar as in feature_tracker.cpp, FeatureTracker::matchStereo

    // Reject stereo match by Essential Matrix
    auto n = cam0_unpts_.size();

    int rejected = 0;
    status_track_.clear();
    status_track_.resize(n, 1);
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d x0(cam0_unpts_[i].x, cam0_unpts_[i].y, 1.);
        Eigen::Vector3d x1(cam1_unpts_[i].x, cam1_unpts_[i].y, 1.);

        Eigen::Vector3d epipolar = data_.essential_c1c0 * x0;
        double epi_err =
            std::abs(x1.transpose() * epipolar) / epipolar.head<2>().norm();
        constexpr double kMaxError = 1.0;
        if (epi_err * data_.focal_length > kMaxError) {
            status_track_[i] = 0;
            rejected++;
        }
    }

    return static_cast<double>(rejected) / total_pts_num_;
}

double StereoCameraVerification::Impl::calcVioEstimatorRejectRate()
{
    auto n = cam0_unpts_.size();

    status_esti1_.clear();
    status_esti1_.resize(n, 1);
    status_esti2_.clear();
    status_esti2_.resize(n, 1);
    status_esti3_.clear();
    status_esti3_.resize(n, 1);

    triangulated_pts_in_cam0_.clear();
    triangulated_pts_in_cam0_.resize(n);

    std::vector<double> depths;
    depths.clear();
    depths.resize(n, -1.);

    int rejected = 0;
    /// reject by negative depth as in feature_manager.cpp
    /// FeatureManager::triangulate
    for (int i = 0; i < n; ++i) {
        const auto &t0 = data_.tic[kCameraLeftId];
        const auto &R0 = data_.ric[kCameraLeftId];
        Matrix34d poseLeft;
        poseLeft.block<3, 3>(0, 0) = R0.transpose();
        poseLeft.block<3, 1>(0, 3) = -R0.transpose() * t0;

        const auto &t1 = data_.tic[kCameraRightId];
        const auto &R1 = data_.ric[kCameraRightId];
        Matrix34d poseRight;
        poseRight.block<3, 3>(0, 0) = R1.transpose();
        poseRight.block<3, 1>(0, 3) = -R1.transpose() * t1;

        const Vector2d ptLeft{cam0_unpts_[i].x, cam0_unpts_[i].y};
        const Vector2d ptRight{cam1_unpts_[i].x, cam1_unpts_[i].y};

        Vector3d point3;
        triangulatePoint(poseLeft, poseRight, ptLeft, ptRight, point3);

        Vector3d left_pt =
            poseLeft.block<3, 3>(0, 0) * point3 + poseLeft.block<3, 1>(0, 3);
        triangulated_pts_in_cam0_[i] = {left_pt.x(), left_pt.y(), left_pt.z()};

        depths[i] = left_pt.z();
        if (depths[i] <= 0.) {
            status_esti1_[i] = 0;
            rejected++;
        }
    }

    /// reject by reprojection as in estimator.cpp
    for (int i = 0; i < n; ++i) {
        if (status_esti1_[i] == 0)
            continue;

        double tmp_error = reprojectionError(
            Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), data_.ric[0],
            data_.tic[0], Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
            data_.ric[1], data_.tic[1], depths[i],
            Eigen::Vector3d(cam0_unpts_[i].x, cam0_unpts_[i].y, 1.),
            Eigen::Vector3d(cam1_unpts_[i].x, cam1_unpts_[i].y, 1.));

        /// Estimator::outliersRejection
        if (tmp_error * data_.focal_length > 3.) {
            status_esti2_[i] = 0;
            rejected++;
        }

        /// Estimator::optimization ByChi2 5.991
        const double info =
            (data_.focal_length / 1.5) * (data_.focal_length / 1.5);
        constexpr double chi2 = 5.991;
        if (tmp_error * tmp_error * info > chi2) {
            status_esti3_[i] = 0;
            if (status_esti2_[i] == 1) {
                rejected++;
            }
        }
    }

    return static_cast<double>(rejected) / total_pts_num_;
}

double StereoCameraVerification::Impl::solveStereoPose()
{
    Vector3d tvec_lw(0., 0., 0.5);
    Quaterniond quat_lw(1., 0., 0., 0.);

    double q[4];
    ceres::RotationMatrixToQuaternion(data_.rot_c1c0.data(), q);
    const Quaterniond quat_r_l(q[0], q[1], q[2], q[3]);
    const Vector3d &tvec_r_l = data_.t_c1c0;

    ceres::Problem problem;
    for (size_t i{0}; i < chessboard_pos_.size(); i++) {
        const Vector3d objPoint(chessboard_pos_[i].x, chessboard_pos_[i].y,
                                chessboard_pos_[i].z);
        const Vector2d leftImgPoint(cam0_unpts_[i].x, cam0_unpts_[i].y);
        const Vector2d rightImgPoint(cam1_unpts_[i].x, cam1_unpts_[i].y);

        auto *costFunc = createStereoReprojectionError1(
            objPoint, leftImgPoint, rightImgPoint, quat_r_l, tvec_r_l,
            data_.focal_length);

        problem.AddResidualBlock(costFunc, nullptr, quat_lw.coeffs().data(),
                                 tvec_lw.data());
    }
    problem.SetManifold(quat_lw.coeffs().data(),
                        new ceres::EigenQuaternionManifold);

    ceres::Solver::Options options;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    q[0] = quat_lw.w();
    q[1] = quat_lw.x();
    q[2] = quat_lw.y();
    q[3] = quat_lw.z();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rcw;
    ceres::QuaternionToRotation(q, rcw.data());

    chess_pts_in_cam0_by_opti_pose_.clear();
    for (int i{0}; i < chessboard_pos_.size(); i++) {
        Vector3d pc = rcw * Vector3d(chessboard_pos_[i].x, chessboard_pos_[i].y,
                                     chessboard_pos_[i].z) +
                      tvec_lw;
        chess_pts_in_cam0_by_opti_pose_.emplace_back(pc.x(), pc.y(), pc.z());
    }

    return summary.final_cost;
}

///------- StereoCameraVerification starts from here
StereoCameraVerification::StereoCameraVerification(const VerifyData &data,
                                                   const Options &options)
    : d(std::make_unique<Impl>(data))
{
    d->init(options);
}

StereoCameraVerification::~StereoCameraVerification() = default;

StereoCameraVerification::Summary StereoCameraVerification::startVerify(
    const cv::Mat &left, const cv::Mat &right)
{
    using Error = StereoCameraVerification::Summary::Error;

    Summary summary;

    // 0. Check sample and verify data
    if (left.empty() || right.empty()) {
        summary.error = Error::InvalidImage;
        LOG(ERROR) << "Empty input images.";
        return summary;
    }

    // TODO: Delete later
    d->left_img_ = left.clone();
    d->right_img_ = right.clone();

    // 1. Find corners
    // TODO: Use CalibBoardBase
    auto findChessboardCorners = [](cv::InputArray img,
                                    const cv::Size &patternSize,
                                    cv::OutputArray corners) -> bool {

#if CV_VERSION_MAJOR == 3
    // TODO
#elif CV_VERSION_MAJOR == 4
        constexpr int flags = cv::CALIB_CB_ACCURACY;

        return cv::findChessboardCornersSB(img, patternSize, corners, flags);
#endif
    };

    if (!findChessboardCorners(left, d->opts_.chessPatternSize, d->cam0_pts_) ||
        !findChessboardCorners(right, d->opts_.chessPatternSize,
                               d->cam1_pts_)) {
        summary.error = Error::FailedFindChessboardCorner;
        return summary;
    }

    d->reorderPoints(d->cam0_pts_);
    d->reorderPoints(d->cam1_pts_);

    // 3. Undistort points
    d->undistortPoints();

    // 4. Reject points by the criteria in mower_vslam_vio module
    summary.trackerRejectRate = d->calcVioTrackerRejectRate();
    if (!std::islessequal(summary.trackerRejectRate,
                          d->opts_.reference.maxTrackerRejectRate)) {
        summary.error |= Error::RejectByTracker;
    }
    summary.estimatorRejectRate = d->calcVioEstimatorRejectRate();
    if (!std::islessequal(summary.estimatorRejectRate,
                          d->opts_.reference.maxEstimatorRejectRate)) {
        summary.error |= Error::RejectByEstimator;
    }

    // 5. Performance point-pair evaluation
    d->real_chess_dis_ = calcPointPairsDistance(d->chessboard_pos_);
    d->triangulated_chess_dis_ =
        calcPointPairsDistance(d->triangulated_pts_in_cam0_);

    summary.stereoOptimzationCost = d->solveStereoPose();
    if (!std::isless(summary.stereoOptimzationCost,
                     d->opts_.reference.maxStereoOptimizationCost)) {
        summary.error |= Error::LargeOptimizationCost;
    }

    summary.rejectRelativeSizeMedian = d->calcRelativeSizeErrMedian();
    if (!std::isless(summary.rejectRelativeSizeMedian,
                     d->opts_.reference.maxRejectRelativeSizeMedian)) {
        summary.error |= Error::LargeRelativeSize;
    }
    summary.rejectRelativePostitionMedian = d->calcRelativePtsPosErrMedian();
    if (!std::isless(summary.rejectRelativePostitionMedian,
                     d->opts_.reference.maxRejectRelativePositionMedian)) {
        summary.error |= Error::LargeRelativePosition;
    }

    return summary;
}

void StereoCameraVerification::drawResult(cv::InputOutputArray verifyResult,
                                          cv::InputOutputArray detectionResult)
{
    if (d->left_img_.empty() || d->right_img_.empty()) {
        return;
    }

    constexpr int kRadius = 2;
    constexpr int kThickness = 2;

    const auto green = CV_RGB(0, 255, 0);
    const auto yellow = CV_RGB(255, 255, 0);
    const auto pink = CV_RGB(205, 90, 106);
    const auto orange = CV_RGB(255, 97, 0);
    const auto navy = CV_RGB(25, 25, 112);

    // Left
    for (const auto &point : d->cam0_pts_) {
        cv::circle(d->left_img_, point, kRadius, green, kThickness);
    }

    // Right
    for (size_t i{0}; i < d->cam1_pts_.size(); i++) {
        const cv::Point rightPt = d->cam1_pts_[i];
        cv::circle(d->right_img_, rightPt, kRadius, green, kThickness);

        // TODO: Ambiguous intention
        // Original writer wanted to avoid points overlap by applying certain
        // offset to each point.
        if (d->status_track_[i] == 0) {
            cv::circle(d->right_img_, rightPt + cv::Point(6, 0), kRadius,
                       yellow, kThickness);
        }
        if (d->status_esti1_[i] == 0) {
            cv::circle(d->right_img_, rightPt + cv::Point(-6, 0), kRadius, pink,
                       kThickness);
        }
        if (d->status_esti2_[i] == 0) {
            cv::circle(d->right_img_, rightPt + cv::Point(0, 6), kRadius,
                       orange, kThickness);
        }
        if (d->status_esti3_[i] == 0) {
            cv::circle(d->right_img_, rightPt + cv::Point(0, -6), kRadius, navy,
                       kThickness);
        }
    }

    cv::hconcat(d->left_img_, d->right_img_, verifyResult);

    if (detectionResult.needed()) {
        cv::Mat left_img_detect, right_img_detect;
        cv::cvtColor(d->left_img_, left_img_detect, cv::COLOR_GRAY2RGB);
        cv::cvtColor(d->right_img_, right_img_detect, cv::COLOR_GRAY2RGB);
        cv::drawChessboardCorners(left_img_detect, d->opts_.chessPatternSize,
                                  d->cam0_pts_, true);
        cv::drawChessboardCorners(right_img_detect, d->opts_.chessPatternSize,
                                  d->cam1_pts_, true);
        cv::hconcat(left_img_detect, right_img_detect, detectionResult);
    }
}

} // namespace tl
