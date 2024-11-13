#pragma once

#include <map>

#include <opencv2/core/mat.hpp>

namespace tl::mcmb {

class BoardGroupObs;
class CameraGroup;

class CameraGroupObs
{
public:
    CameraGroupObs() = delete;
    CameraGroupObs(std::shared_ptr<CameraGroup> camGroup,
                   bool useQuaternionAverage);

    void insertBoardGroupObservation(
        std::shared_ptr<BoardGroupObs> observation);

    void setObjectPose(cv::InputArray pose, int boardGroupId);
    void setObjectPose(cv::Mat rvec, cv::Mat tvec, int boardGroupId);
    void getObjectPose(cv::OutputArray rvec, cv::OutputArray tvec,
                       int boardGroupId) const;
    cv::Mat objectPose(int boardGroupId) const;
    inline auto objectOrientation(int boardGroupId) const
    {
        cv::Mat rvec, _;
        getObjectPose(rvec, _, boardGroupId);
        return rvec;
    }
    inline auto objectPosition(int boardGroupId) const
    {
        cv::Mat _, tvec;
        getObjectPose(_, tvec, boardGroupId);
        return tvec;
    }

    void computeBoardGroupsPose();
    void updateObjObsPose();

public:
    // Objects
    // index of the visible 3d objects
    std::vector<int> _boardGroupIds;
    std::map<int, std::weak_ptr<BoardGroupObs>> _boardGroupObservations;
    // object pose wrt. the ref. cam of the group
    std::map<int, std::array<double, 6>> object_pose_;

    // Camera group
    int _camGroupId;
    std::weak_ptr<CameraGroup> _camGroup;

    bool _useQuaternionAvg = true;
};

} // namespace tl::mcmb
