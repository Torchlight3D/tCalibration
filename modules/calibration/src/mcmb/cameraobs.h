#pragma once

#include <map>
#include <memory>
#include <vector>

namespace tl::mcmb {

class BoardObs;
class BoardGroupObs;
class Camera;

// Brief:
// An observation of camera Id at frame Id is a Camera Observation. So basically
// we can say an image is a Camera Observation. It contains:
// + Boards observed in this image
// + Objects observed in this image
//
// TODO:
// 1. Use CameraObsId to represent std::pair<int, int>
class CameraObs
{
public:
    CameraObs() = delete;
    explicit CameraObs(const std::pair<int, int>& id);

    void insertBoardObservation(std::shared_ptr<BoardObs> observation);
    void insertBoardGroupObservation(
        std::shared_ptr<BoardGroupObs> observation);

public:
    // Boards
    std::vector<int> _boardIds;
    std::map<int, std::weak_ptr<BoardObs>> _boardObservations;

    // Objects
    std::vector<int> _boardGroupIds;
    std::map<int, std::weak_ptr<BoardGroupObs>> _boardGroupObservations;

    // {CameraId -> FrameId}
    std::pair<int, int> _id;
};

} // namespace tl::mcmb
