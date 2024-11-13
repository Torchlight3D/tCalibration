#include "cameragroupobs.h"

#include <opencv2/core.hpp>

#include "boardgroupobs.h"
#include "cameragroup.h"
#include "utils.h"

namespace tl::mcmb {

CameraGroupObs::CameraGroupObs(std::shared_ptr<CameraGroup> camGroup,
                               bool useQuaternionAvg)
    : _camGroup(camGroup),
      _camGroupId(camGroup->_id),
      _useQuaternionAvg(useQuaternionAvg)
{
}

void CameraGroupObs::insertBoardGroupObservation(
    std::shared_ptr<BoardGroupObs> boardGroupObs)
{
    _boardGroupIds.push_back(boardGroupObs->_boardGroupId);
    _boardGroupObservations[_boardGroupObservations.size()] = boardGroupObs;
}

void CameraGroupObs::setObjectPose(cv::InputArray pose, int boardGroupId)
{
    cv::Mat rvec, tvec;
    Proj2RT(pose, rvec, tvec);
    setObjectPose(rvec, tvec, boardGroupId);
}

void CameraGroupObs::setObjectPose(cv::Mat rvec, cv::Mat tvec, int boardGroupId)
{
    object_pose_[boardGroupId] = {rvec.at<double>(0), rvec.at<double>(1),
                                  rvec.at<double>(2), tvec.at<double>(0),
                                  tvec.at<double>(1), tvec.at<double>(2)};
}

void CameraGroupObs::getObjectPose(cv::OutputArray rvec, cv::OutputArray tvec,
                                   int boardGroupId) const
{
    const auto &pose = object_pose_.at(boardGroupId);

    cv::Mat{cv::Vec3d{pose[0], pose[1], pose[2]}}.reshape(1, 3).copyTo(rvec);
    cv::Mat{cv::Vec3d{pose[3], pose[4], pose[5]}}.reshape(1, 3).copyTo(tvec);
}

cv::Mat CameraGroupObs::objectPose(int boardGroupId) const
{
    cv::Mat rvec, tvec;
    getObjectPose(rvec, tvec, boardGroupId);
    return RVecT2Proj(rvec, tvec);
}

void CameraGroupObs::computeBoardGroupsPose()
{
    // Find group of observation of the same object
    std::vector<int> uniqueObjectIds = _boardGroupIds;
    {
        const auto uniq = std::ranges::unique(uniqueObjectIds);
        uniqueObjectIds.erase(uniq.begin(), uniq.end());
    }

    // Find indexes of object 3D obs with common index
    // Object index -> index in the vector of observation
    std::map<int, std::vector<int>> boardGroupIdToObsInds;
    for (const auto &boardGroupId : uniqueObjectIds) {
        for (size_t i = 0; i < _boardGroupIds.size(); i++) {
            if (_boardGroupIds[i] == boardGroupId) {
                boardGroupIdToObsInds[boardGroupId].push_back(i);
            }
        }
    }

    // Compute the pose of the objects:
    for (const auto &[boardGroupId, boardGroupObsInds] :
         boardGroupIdToObsInds) {
        // if the reference camera has an observation, take its pose as initial
        // value
        bool flag_ref_cam = false;
        cv::Mat group_pose_r, group_pose_t;
        for (const auto &boardGroupObsInd : boardGroupObsInds) {
            auto camGroup = _camGroup.lock();
            auto boardGroupObs =
                _boardGroupObservations[boardGroupObsInd].lock();
            if (camGroup && boardGroupObs &&
                camGroup->_refCamId == boardGroupObs->_cameraId) {
                flag_ref_cam = true;
                group_pose_r = boardGroupObs->orientationInGroup();
                group_pose_t = boardGroupObs->positionInGroup();
            }
        }

        // if the reference camera has no visible observation, then take
        // the average of other observations
        if (!flag_ref_cam) {
            std::vector<double> r1, r2, r3;
            cv::Mat average_translation = cv::Mat::zeros(3, 1, CV_64F);
            for (const auto &boardGroupObsInd : boardGroupObsInds) {
                auto boardGroupObs =
                    _boardGroupObservations[boardGroupObsInd].lock();
                if (!boardGroupObs) {
                    continue;
                }

                const cv::Mat rvec = boardGroupObs->orientationInGroup();
                r1.push_back(rvec.at<double>(0));
                r2.push_back(rvec.at<double>(1));
                r3.push_back(rvec.at<double>(2));
                average_translation += boardGroupObs->positionInGroup();
            }

            // Average version
            group_pose_t = average_translation / boardGroupObsInds.size();
            group_pose_r = calcAverageRotation(r1, r2, r3, _useQuaternionAvg);
        }

        // set the pose and update the observation
        setObjectPose(group_pose_r, group_pose_t, boardGroupId);
        for (const auto &boardGroupObsInd : boardGroupObsInds) {
            if (auto objObs =
                    _boardGroupObservations[boardGroupObsInd].lock()) {
                objObs->setPoseInGroup(group_pose_r, group_pose_t);
            }
        }
    }
}

void CameraGroupObs::updateObjObsPose()
{
    for (const auto &[_, _boardGroupObs] : _boardGroupObservations) {
        auto boardGroupObs = _boardGroupObs.lock();
        if (!boardGroupObs) {
            continue;
        }

        cv::Mat rvec, tvec;
        getObjectPose(rvec, tvec, boardGroupObs->_boardGroupId);
        boardGroupObs->setPoseInGroup(rvec, tvec);
    }
}

} // namespace tl::mcmb
