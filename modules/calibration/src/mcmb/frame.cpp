#include "frame.h"

#include "boardobs.h"
#include "boardgroupobs.h"
#include "cameraobs.h"

namespace tl::mcmb {

Frame::Frame(int frameId, double timestamp) : _stamp(timestamp), _id(frameId) {}

void Frame::insertObservation(int camId, const std::string& imagePath)
{
    _imagePaths.insert({camId, imagePath});
}

void Frame::insertBoardObservation(std::shared_ptr<BoardObs> board)
{
    _boardIds.push_back(board->_boardId);
    _boardObservations[_boardObservations.size()] = board;
}

void Frame::insertBoardGroupObservation(std::shared_ptr<BoardGroupObs> obs)
{
    _boardGroupIds.push_back(obs->_boardGroupId);
    _boardGroupObservations[_boardGroupObservations.size()] = obs;
}

void Frame::insertCameraObservation(std::shared_ptr<CameraObs> obs)
{
    if (obs->_id.second == _id) {
        _cameraIds.push_back(obs->_id.first);
        _cameraObservations[_cameraObservations.size()] = obs;
    }
}

void Frame::insertCameraGroupObservation(std::shared_ptr<CameraGroupObs> obs,
                                         int id)
{
    _camGroupIds.push_back(id);
    _camGroupObservations[_camGroupObservations.size()] = obs;
}

} // namespace tl::mcmb
