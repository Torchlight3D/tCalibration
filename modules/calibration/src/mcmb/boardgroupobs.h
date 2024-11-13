#pragma once

#include <map>

#include <opencv2/core/mat.hpp>

namespace tl::mcmb {

class Board;
class BoardObs;
class BoardGroup;
class Camera;

class BoardGroupObs
{
public:
    BoardGroupObs() = delete;
    BoardGroupObs(std::shared_ptr<BoardGroup> boardGroup, int boardGroupId);

    void insertBoardObservation(std::shared_ptr<BoardObs> observation);

    void setPose(cv::InputArray pose);
    void setPose(const cv::Mat rvec, const cv::Mat tvec);
    void getPose(cv::OutputArray rvec, cv::OutputArray tvec) const;
    cv::Mat pose() const;
    inline auto orientation() const
    {
        cv::Mat rvec, _;
        getPose(rvec, _);
        return rvec;
    }
    inline auto position() const
    {
        cv::Mat _, tvec;
        getPose(_, tvec);
        return tvec;
    }

    void setPoseInGroup(cv::InputArray pose);
    void setPoseInGroup(const cv::Mat rvec, const cv::Mat tvec);
    void getPoseInGroup(cv::OutputArray rvec, cv::OutputArray tvec) const;
    cv::Mat poseInGroup() const;
    inline auto orientationInGroup() const
    {
        cv::Mat rvec, _;
        getPoseInGroup(rvec, _);
        return rvec;
    }
    inline auto positionInGroup() const
    {
        cv::Mat _, tvec;
        getPoseInGroup(_, tvec);
        return tvec;
    }

    void estimatePose(float threshold, int iterations);
    float calcRpeMean() const;

public:
    int _frameId;
    int _cameraId;

    // TODO: Not used???
    std::vector<int> _boardIds;

    std::array<double, 6> _pose;

    // Pose of the object expressed in camera group referential
    std::array<double, 6> _groupPose;

    // points
    std::vector<cv::Point2f> _pixels;
    std::vector<int> _pointInds;

    std::weak_ptr<Camera> _camera;
    std::weak_ptr<BoardGroup> _boardGroup;

    std::map<int, std::weak_ptr<Board>> _boards;
    std::map<int, std::weak_ptr<BoardObs>> _boardObservations;

    int _boardGroupId;

    // TODO: Not used???
    bool _valid = true;
};

} // namespace tl::mcmb
