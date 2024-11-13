#include "boardgroupobs.h"

#include <glog/logging.h>

#include "boardgroup.h"
#include "boardobs.h"
#include "camera.h"
#include "utils.h"

namespace tl::mcmb {

BoardGroupObs::BoardGroupObs(std::shared_ptr<BoardGroup> boardGroup,
                             int boardGroupId)
    : _boardGroup(boardGroup), _boardGroupId(boardGroupId)
{
}

void BoardGroupObs::insertBoardObservation(std::shared_ptr<BoardObs> boardObs)
{
    _boardIds.push_back(boardObs->_boardId);
    _camera = boardObs->_camera;
    _cameraId = boardObs->_cameraId;
    _frameId = boardObs->_frameId;
    _boardObservations[_boardObservations.size()] = boardObs;

    const size_t num_points = boardObs->_corners.size();
    _pixels.reserve(num_points);
    _pointInds.reserve(num_points);
    for (size_t i = 0; i < num_points; i++) {
        // Convert the index from the board to the object
        std::pair<int, int> boardIdCornerId =
            std::make_pair(boardObs->_boardId, boardObs->_cornerIds[i]);
        if (auto boardGroup = _boardGroup.lock()) {
            _pixels.emplace_back(boardObs->_corners[i]);
            _pointInds.emplace_back(
                boardGroup->_boardIdCornerIdToObjId[boardIdCornerId]);
        }
    }
}

void BoardGroupObs::setPose(cv::InputArray pose)
{
    cv::Mat rvec, tvec;
    Proj2RT(pose, rvec, tvec);
    setPose(rvec, tvec);
}

void BoardGroupObs::setPose(const cv::Mat rvec, const cv::Mat tvec)
{
    _pose = {rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
             tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
}

void BoardGroupObs::getPose(cv::OutputArray rvec, cv::OutputArray tvec) const
{
    cv::Mat{cv::Vec3d{_pose[0], _pose[1], _pose[2]}}.reshape(1, 3).copyTo(rvec);
    cv::Mat{cv::Vec3d{_pose[3], _pose[4], _pose[5]}}.reshape(1, 3).copyTo(tvec);
}

cv::Mat BoardGroupObs::pose() const
{
    cv::Mat rvec, tvec;
    getPose(rvec, tvec);
    return RVecT2Proj(rvec, tvec);
}

void BoardGroupObs::setPoseInGroup(cv::InputArray pose)
{
    cv::Mat rvec, tvec;
    Proj2RT(pose, rvec, tvec);
    setPoseInGroup(rvec, tvec);
}

void BoardGroupObs::setPoseInGroup(const cv::Mat rvec, const cv::Mat tvec)
{
    _groupPose = {rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
                  tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
}

void BoardGroupObs::getPoseInGroup(cv::OutputArray rvec,
                                   cv::OutputArray tvec) const
{
    cv::Mat{cv::Vec3d{_groupPose[0], _groupPose[1], _groupPose[2]}}
        .reshape(1, 3)
        .copyTo(rvec);
    cv::Mat{cv::Vec3d{_groupPose[3], _groupPose[4], _groupPose[5]}}
        .reshape(1, 3)
        .copyTo(tvec);
}

cv::Mat BoardGroupObs::poseInGroup() const
{
    cv::Mat rvec, tvec;
    getPoseInGroup(rvec, tvec);
    return RVecT2Proj(rvec, tvec);
}

void BoardGroupObs::estimatePose(float threshold, int iterations)
{
    auto cam = _camera.lock();
    auto boardGroup = _boardGroup.lock();
    if (!cam || !boardGroup) {
        return;
    }

    std::vector<cv::Point3f> objPoints;
    objPoints.reserve(_pointInds.size());
    for (const auto &pointInd : _pointInds) {
        objPoints.push_back(boardGroup->_points[pointInd]);
    }

    cv::Mat rvec, tvec;
    const cv::Mat inliers = solveP3PRansac(
        objPoints, _pixels, cam->cameraMatrix(), cam->distortion(), rvec, tvec,
        threshold, iterations, cam->_type);

    // LOG(INFO) << "Translation: " << tvec << ", Rotation: " << rvec;
    // LOG(INFO) << "input pts 3D :: " << object_pts_temp.size();
    // LOG(INFO) << "Inliers: " << inliers.rows;

    setPose(rvec, tvec);
}

float BoardGroupObs::calcRpeMean() const
{
    auto cam = _camera.lock();
    auto boardGroup = _boardGroup.lock();
    if (!cam || !boardGroup) {
        return 0.f;
    }

    std::vector<cv::Point3f> objPoints;
    objPoints.reserve(_pointInds.size());
    for (const auto &pointInd : _pointInds) {
        objPoints.push_back(boardGroup->_points[pointInd]);
    }

    std::vector<cv::Point2f> reprojs;
    projectPointsWithDistortion(objPoints, orientation(), position(),
                                cam->cameraMatrix(), cam->distortion(), reprojs,
                                cam->_type);

    auto rpe{0.f};
    std::vector<float> diffs;
    diffs.reserve(_pointInds.size());
    for (size_t i{0}; i < _pointInds.size(); i++) {
        const auto diff = cv::norm(_pixels[i] - reprojs[i]);
        diffs.push_back(diff);
        rpe += diff;
    }
    rpe /= diffs.size();

    return rpe;
}

} // namespace tl::mcmb
