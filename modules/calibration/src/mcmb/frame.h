#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tl::mcmb {

class BoardObs;
class CameraObs;
class CameraGroupObs;
class BoardGroupObs;

// Brief:
// A Frame is all the observations from all the (synchronized) cameras in the
// system at a given time.
//
// e.g. If we have a stereo camera to calibrate, and they look at three boards.
// In every observation, they can see part of the all three boards. Then an
// ideal frame contains:
// + 2 camera Ids (0, 1)
// + 2 images (' file path)
// + 6 BoardObservations
// + ??? ObjectObservations
// + ??? CameraObservations
// + ??? CameraGroupObservations
//
// TODO:
// 1. Consider change name.
// 2. Drop image file and camera id in ctor parameter.
// 3. Add a method to add new observation
class Frame
{
public:
    Frame() = delete;
    Frame(int frameId, double timestamp);

    void insertObservation(int camId, const std::string& imagePath);
    void insertBoardObservation(std::shared_ptr<BoardObs> observation);
    void insertBoardGroupObservation(
        std::shared_ptr<BoardGroupObs> observation);
    void insertCameraObservation(std::shared_ptr<CameraObs> observation);
    void insertCameraGroupObservation(
        std::shared_ptr<CameraGroupObs> observation, int camGroupId);

public:
    // Images
    // {CameraId -> imagePath}
    std::map<int, std::string> _imagePaths;

    // Camera observations
    // TODO: Not one use???
    std::vector<int> _cameraIds;
    std::map<int, std::weak_ptr<CameraObs>> _cameraObservations;

    // Camera Group observations
    std::vector<int> _camGroupIds;
    std::map<int, std::weak_ptr<CameraGroupObs>> _camGroupObservations;

    // Board observations
    std::vector<int> _boardIds;
    std::map<int, std::weak_ptr<BoardObs>> _boardObservations;

    // Object observations
    std::vector<int> _boardGroupIds;
    std::map<int, std::weak_ptr<BoardGroupObs>> _boardGroupObservations;

    double _stamp;
    int _id;
};

} // namespace tl::mcmb
