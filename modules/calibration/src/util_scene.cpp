#include "util_scene.h"

#include <tCore/TimeUtils>
#include <tVision/Target/CalibBoardBase>

namespace tl {

namespace mvs {

void setupBoardInScene(Scene::Ptr scene, const CalibBoardBase &board)
{
    if (!scene) {
        return;
    }

    // Clear existed board points (track)
    const auto trackIds = scene->trackIds();
    for (const auto &trackId : trackIds) {
        scene->removeTrack(trackId);
    }

    const auto &boardPoints = board.boardPoints();

    Vector3dList objectPoints;
    objectPoints.reserve(boardPoints.size());
    std::transform(boardPoints.begin(), boardPoints.end(),
                   std::back_inserter(objectPoints), [](const auto &pt) {
                       return Eigen::Vector3d{pt.x, pt.y, pt.z};
                   });

    for (int i{0}; i < board.cornerCount(); ++i) {
        const auto &trackId = board.cornerIds()[i];
        scene->addTrack(trackId);
        auto *track = scene->rTrack(trackId);
        track->setEstimated(true);
        track->rPosition() = objectPoints[i].homogeneous();
    }
}

std::string makeUniqueViewName(CameraId camId, double timestamp)
{
    // TODO: Need a UUID-like generator
    return std::to_string(time::sToNs(timestamp)) + "_" + std::to_string(camId);
}

} // namespace mvs

} // namespace tl
