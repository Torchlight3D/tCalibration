#include "boardobs.h"

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "board.h"
#include "camera.h"
#include "utils.h"

namespace tl::mcmb {

BoardObs::BoardObs(int camId, int frameId, int boardId,
                   const std::vector<cv::Point2f> &corners,
                   const std::vector<int> &cornerIds,
                   std::shared_ptr<Camera> camera, std::shared_ptr<Board> board)
    : _frameId(frameId),
      _cameraId(camId),
      _boardId(boardId),
      _corners(corners),
      _cornerIds(cornerIds),
      _camera(camera),
      _board(board)
{
    // CHECK(camera) << "Invalid camera to create Board Observation";
    // CHECK(board) << "Invalid board to create Board Observation";
}

void BoardObs::setPose(cv::InputArray pose)
{
    cv::Mat rvec, tvec;
    Proj2RT(pose, rvec, tvec);
    setPose(rvec, tvec);
}

void BoardObs::setPose(const cv::Mat rvec, const cv::Mat tvec)
{
    _pose = {rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
             tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
}

void BoardObs::getPose(cv::OutputArray rvec, cv::OutputArray tvec) const
{
    cv::Mat{cv::Vec3d{_pose[0], _pose[1], _pose[2]}}.reshape(1, 3).copyTo(rvec);
    cv::Mat{cv::Vec3d{_pose[3], _pose[4], _pose[5]}}.reshape(1, 3).copyTo(tvec);
}

cv::Mat BoardObs::pose() const
{
    cv::Mat rvec, tvec;
    getPose(rvec, tvec);
    return RVecT2Proj(rvec, tvec);
}

void BoardObs::estimatePose(float threshold, int iterations)
{
    auto cam = _camera.lock();
    auto board = _board.lock();
    if (!cam || !board) {
        return;
    }

    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(_cornerIds.size());
    for (const auto &cornerId : _cornerIds) {
        objectPoints.push_back(board->point(cornerId));
    }

    cv::Mat rvec, tvec;
    const cv::Mat inliers = solveP3PRansac(
        objectPoints, _corners, cam->cameraMatrix(), cam->distortion(), rvec,
        tvec, threshold, iterations, cam->_type);

    setPose(rvec, tvec);

    const auto &numInliers = inliers.rows;

    // LOG(INFO) << "Estimated relative pose from board " << board_id_
    //           << " to camera " << camera_id_;
    // LOG(INFO) << "Translation: " << tvec << ", Rotation: " << rvec;
    // LOG(INFO) << "With " << _cornerIds.size() << " 2D-3D point pairs, "
    //           << numInliers << " inliers.";

    constexpr auto kMinNumInliers{4};
    _valid = numInliers >= kMinNumInliers;

    // Remove outliers
    std::vector<cv::Point2f> newCorners;
    std::vector<int> newCornerIds;
    newCorners.reserve(numInliers);
    newCornerIds.reserve(numInliers);
    for (auto i{0}; i < numInliers; i++) {
        const auto index = inliers.at<int>(i);
        newCorners.push_back(_corners[index]);
        newCornerIds.push_back(_cornerIds[index]);
    }
    _corners = newCorners;
    _cornerIds = newCornerIds;
}

float BoardObs::calcRpeMean() const
{
    auto cam = _camera.lock();
    auto board = _board.lock();
    if (!cam || !board) {
        return -1.f;
    }

    const auto numPoints = _cornerIds.size();

    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(numPoints);
    for (const auto &cornerId : _cornerIds) {
        objectPoints.push_back(board->point(cornerId));
    }

    std::vector<cv::Point2f> reprojs;
    projectPointsWithDistortion(objectPoints, orientation(), position(),
                                cam->cameraMatrix(), cam->distortion(), reprojs,
                                cam->_type);

    auto rpe{0.f};
    std::vector<float> diffs;
    diffs.reserve(numPoints);
    for (size_t i{0}; i < numPoints; ++i) {
        const auto diff = cv::norm(_corners[i] - reprojs[i]);
        diffs.push_back(diff);
        rpe += diff;
    }
    rpe /= numPoints;

    constexpr auto kMaxBoardObsRpe{1.f};
    if (rpe > kMaxBoardObsRpe) {
        LOG(WARNING) << "Detect high RPE in current board pose estimation: "
                     << "Frame " << _frameId << ", Board " << _boardId
                     << ", RPE Mean: " << rpe << " of point count "
                     << numPoints;
    }

    return rpe;
}

} // namespace tl::mcmb
