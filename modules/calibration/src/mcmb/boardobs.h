#pragma once

#include <opencv2/core/mat.hpp>

namespace tl::mcmb {

class Board;
class Camera;

// A Board Observation contains the 2D points and corresponding 3D points of an
// observed board. Additionally, it contains the pose of the board w.r.t. the
// camera observing the board.
// Soft link to:
// + The Camera observing the Board
// + The observed Board
class BoardObs
{
public:
    BoardObs() = delete;
    BoardObs(int cameraId, int frameId, int boardId,
             const std::vector<cv::Point2f> &corners,
             const std::vector<int> &cornerIds, std::shared_ptr<Camera> camera,
             std::shared_ptr<Board> board);

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

    // Estimate the pose of this board w.r.t. the camera observing it.
    void estimatePose(float threshold, int iterations);
    float calcRpeMean() const;

public:
    int _frameId;
    int _cameraId;
    int _boardId;

    // Key instances
    std::array<double, 6> _pose;
    std::vector<cv::Point2f> _corners;
    std::vector<int> _cornerIds;

    std::weak_ptr<Camera> _camera;
    std::weak_ptr<Board> _board;

    // Validity : An observation is judged not valid when the RANSAC pose
    // estimation return too few pts
    bool _valid = true;
};

} // namespace tl::mcmb
