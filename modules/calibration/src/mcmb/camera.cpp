#include "camera.h"

#include <random>

#include <glog/logging.h>
#include <opencv2/calib3d.hpp>

#include "board.h"
#include "boardobs.h"
#include "boardgroupobs.h"
#include "factors.h"
#include "frame.h"

namespace tl::mcmb {

namespace {
constexpr auto kParamOffset = 4;
} // namespace

Camera::Camera(int camId, int type) : _id(camId), _type(type) {}

void Camera::insertFrame(std::shared_ptr<Frame> frame)
{
    if (const auto &frameId = frame->_id; !_frames.contains(frameId)) {
        _frames.insert({frameId, frame});
    }
}

void Camera::insertBoardObservation(std::shared_ptr<BoardObs> boardObs)
{
    _boardObservations[_boardObservations.size()] = boardObs;
    _visBoardIds.push_back(boardObs->_boardId);
}

void Camera::insertBoardGroupObservation(
    std::shared_ptr<BoardGroupObs> boardGroupObs)
{
    _boardGroupObservations[_boardGroupObservations.size()] = boardGroupObs;
    _visBoardGroupIds.push_back(boardGroupObs->_boardGroupId);
}

void Camera::clear()
{
    // ...
}

cv::Mat Camera::cameraMatrix() const
{
    cv::Mat K = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
    K.at<double>(0, 0) = intrinsics_[0];
    K.at<double>(1, 1) = intrinsics_[1];
    K.at<double>(0, 2) = intrinsics_[2];
    K.at<double>(1, 2) = intrinsics_[3];
    K.at<double>(2, 2) = 1.;
    return K;
}

void Camera::setCameraMatrix(const cv::Mat K)
{
    intrinsics_[0] = K.at<double>(0, 0);
    intrinsics_[1] = K.at<double>(1, 1);
    intrinsics_[2] = K.at<double>(0, 2);
    intrinsics_[3] = K.at<double>(1, 2);
}

void Camera::setDistortion(const cv::Mat distortion)
{
    // First 4 values are projective parameters [fx, fy, cx, cy]
    switch (_type) {
        case 0: {
            for (auto i = kParamOffset; i < 9; ++i) {
                intrinsics_[i] = distortion.at<double>(i - kParamOffset);
            }
            break;
        }
        case 1: {
            for (auto i = kParamOffset; i < 8; ++i) {
                intrinsics_[i] = distortion.at<double>(i - kParamOffset);
            }
            break;
        }
        default:
            CV_Error(cv::Error::StsBadFlag, "Unsupported camera model!");
            break;
    }
}

cv::Mat Camera::distortion() const
{
    switch (_type) {
        case 0: {
            cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
            for (auto i = kParamOffset; i < 9; ++i) {
                distortion.at<double>(i - kParamOffset) = intrinsics_[i];
            }
            return distortion;
        }
        case 1: {
            cv::Mat distortion = cv::Mat::zeros(1, 4, CV_64F);
            for (auto i = kParamOffset; i < 8; ++i) {
                distortion.at<double>(i - kParamOffset) = intrinsics_[i];
            }
            return distortion;
        }
        default: {
            CV_Error(cv::Error::StsBadFlag, "Unsupported camera model!");
            break;
        }
    }
}

void Camera::initializeCalibration()
{
    LOG(INFO) << "Camera " << _id << " has " << _boardObservations.size()
              << " board observations from " << _frames.size() << " frames";

    std::vector<size_t> boardObsIndices(_boardObservations.size());
    std::iota(boardObsIndices.begin(), boardObsIndices.end(), size_t(0));

    std::random_device rd{};
    std::mt19937_64 rng{rd()};
    std::ranges::shuffle(boardObsIndices, rng);

    // Fisheye is more sensitive and require more images.
    constexpr size_t kMinViews{50};
    constexpr size_t kMinFisheyeViews{500};
    const auto numUsedBoardObs = std::min(
        boardObsIndices.size(), (_type == 1 ? kMinFisheyeViews : kMinViews));

    // Prepare 2D-3D correspondences
    std::vector<std::vector<cv::Point3f>> objPointsList;
    std::vector<std::vector<cv::Point2f>> imgPointsList;
    objPointsList.reserve(numUsedBoardObs);
    imgPointsList.reserve(numUsedBoardObs);
    for (size_t i{0}; i < numUsedBoardObs; ++i) {
        auto boardObs = _boardObservations[boardObsIndices[i]].lock();
        if (!boardObs) {
            continue;
        }

        // if fisheye, check if points are not too close to borders
        bool validBoardObs{true};
        if (_type == 1) {
            validBoardObs = !checkPointsCloseToBorder(boardObs);
        }

        if (!validBoardObs) {
            continue;
        }

        auto board = boardObs->_board.lock();
        if (!board) {
            continue;
        }

        const auto &cornerIds = boardObs->_cornerIds;
        std::vector<cv::Point3f> objPoints;
        objPoints.reserve(cornerIds.size());
        for (const auto &cornerId : cornerIds) {
            objPoints.push_back(board->point(cornerId));
        }

        objPointsList.push_back(objPoints);
        imgPointsList.push_back(boardObs->_corners);
    }

    // Initialize with OpenCV
    cv::Mat cameraMatrix, distortion, rvec, tvec;
    double rpe_rms{0.};
    switch (_type) {
        case 0: {
            rpe_rms =
                cv::calibrateCamera(objPointsList, imgPointsList, _imgSize,
                                    cameraMatrix, distortion, rvec, tvec);
            break;
        };
        case 1: {
            // NOTE: Workaround for OpenCV fisheye init intrinsics bug.
            // Remove after upgrading OpenCV 4.10
            const auto f_init = std::max(_imgSize.width, _imgSize.height) / 2.;
            const auto [cx_init, cy_init] =
                cv::Size2d{_imgSize} / 2. - cv::Size2d{0.5, 0.5};
            // clang-format off
            cameraMatrix = (cv::Mat_<double>(3, 3) << f_init, 0., cx_init,
                                                      0., f_init, cy_init,
                                                      0.,     0.,     1.);
            // clang-format on
            distortion = (cv::Mat_<double>(1, 4) << 0., 0., 0., 0.);
            rpe_rms = cv::fisheye::calibrate(
                objPointsList, imgPointsList, _imgSize, cameraMatrix,
                distortion, rvec, tvec,
                cv::fisheye::CALIB_USE_INTRINSIC_GUESS |
                    cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                    cv::fisheye::CALIB_FIX_SKEW);
            break;
        };
        default:
            CV_Error(cv::Error::StsBadFlag, "Unsupported camera model!");
            break;
    }

    LOG(INFO) << "Initial projection: " << cameraMatrix;
    LOG(INFO) << "Initial distortion: " << distortion;
    LOG(INFO) << "With RPE RMS: " << rpe_rms;

    setIntrinsics(cameraMatrix, distortion);
}

void Camera::refineIntrinsicCalibration(int numIterations)
{
    LOG(INFO) << "Projection before optimization: " << cameraMatrix();
    LOG(INFO) << "Distortion before optimization: " << distortion();

    ceres::Problem problem;
    for (const auto &[_0, _boardObs] : _boardObservations) {
        auto boardObs = _boardObs.lock();
        if (!boardObs || !boardObs->_valid) {
            continue;
        }

        auto board = boardObs->_board.lock();
        if (!board) {
            continue;
        }

        const auto &objPoints = board->points();
        const auto &cornerIds = boardObs->_cornerIds;
        const auto &imgPoints = boardObs->_corners;
        for (size_t i{0}; i < cornerIds.size(); ++i) {
            const auto &objPoint = objPoints[cornerIds[i]];
            const auto &imgPoint = imgPoints[i];
            auto *cost = ReprojectionError::create(
                double(imgPoint.x), double(imgPoint.y), double(objPoint.x),
                double(objPoint.y), double(objPoint.z), _type);
            problem.AddResidualBlock(cost, new ceres::HuberLoss(1.),
                                     boardObs->_pose.data(),
                                     intrinsics_.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = numIterations;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    LOG(INFO) << "Projection after optimization: " << cameraMatrix();
    LOG(INFO) << "Distortion after optimization: " << distortion();
}

bool Camera::checkPointsCloseToBorder(std::shared_ptr<BoardObs> boardObs) const
{
    constexpr auto kBorderRatio = 0.05f;

    const auto borderSize =
        cv::Vec2f(_imgSize.width, _imgSize.height) * kBorderRatio;

    const cv::Rect2f roi{cv::Point2f{borderSize},
                         cv::Size2f{_imgSize} - cv::Size2f{borderSize * 2.f}};

    return std::ranges::any_of(boardObs->_corners, [&roi](const auto &point) {
        return !roi.contains(point);
    });
}

} // namespace tl::mcmb
