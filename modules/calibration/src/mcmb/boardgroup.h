#pragma once

#include <map>

#include <opencv2/core/mat.hpp>

namespace tl::mcmb {

class Board;
class BoardObs;
class BoardGroupObs;
class Camera;
class Frame;

// BoardGroup is a collection of Boards.
// Isolated collection of Boards forms a BoardGroup.
// e.g. 5 Boards and 3 Cameras in the scene. 2 of the boards in the front and 3
// of them in the back, while 2 cameras look forward, 1 camera looks backward.
class BoardGroup
{
public:
    BoardGroup() = delete;
    BoardGroup(int refBoardId, int id);

    void insertFrame(std::shared_ptr<Frame> frame);
    void insertBoard(std::shared_ptr<Board> board);
    void insertObservation(std::shared_ptr<BoardGroupObs> observation);

    void setBoardPose(cv::Mat pose, int boardId);
    void setBoardPose(cv::Mat rvec, cv::Mat tvec, int boardId);
    void getBoardPose(cv::Mat &rvec, cv::Mat &tvec, int boardId) const;
    cv::Mat boardPose(int boardId) const;
    inline auto boardOrientation(int boardId) const
    {
        cv::Mat rvec, _;
        getBoardPose(rvec, _, boardId);
        return rvec;
    }
    inline auto boardPosition(int boardId) const
    {
        cv::Mat _, tvec;
        getBoardPose(_, tvec, boardId);
        return tvec;
    }

    void refineBoardGroup(int iterations);
    void updateObjectPoints();

public:
    // Parameters
    int _refBoardId;
    int _id;
    size_t _numPoints;

    // 3D points
    std::vector<cv::Point3f> _points;

    // [Board Id, Corner Id] -> Object Point Index.
    // The Corner Id in Board Id corresponds to object point index
    std::map<std::pair<int, int>, int> _boardIdCornerIdToObjId;
    // Object Point Index -> [Board Id, Corner Id]
    std::vector<std::pair<int, int>> _boardIdCornerId;

    // Boards composing the object
    std::map<int, std::weak_ptr<Board>> _boards;
    std::map<int, std::array<double, 6>> _relBoardPoses;

    // List of object observation for this 3D object
    std::map<int, std::weak_ptr<BoardGroupObs>> _observations;
    std::map<int, std::weak_ptr<Frame>> _frames;

private:
};

} // namespace tl::mcmb
