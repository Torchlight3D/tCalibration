#include "cameraobs.h"

#include "boardobs.h"
#include "boardgroupobs.h"

namespace tl::mcmb {

CameraObs::CameraObs(const std::pair<int, int>& id) : _id(id) {}

void CameraObs::insertBoardObservation(std::shared_ptr<BoardObs> boardObs)
{
    if (boardObs->_cameraId == _id.first) {
        _boardObservations[_boardObservations.size()] = boardObs;
        _boardIds.push_back(boardObs->_boardId);
    }
}

void CameraObs::insertBoardGroupObservation(
    std::shared_ptr<BoardGroupObs> boardGroupObs)
{
    if (boardGroupObs->_cameraId == _id.first) {
        _boardGroupObservations[_boardGroupObservations.size()] = boardGroupObs;
        _boardGroupIds.push_back(boardGroupObs->_boardGroupId);
    }
}

} // namespace tl::mcmb
