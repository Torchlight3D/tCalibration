#pragma once

#include <map>

#include <opencv2/core/mat.hpp>

namespace tl::mcmb {

class BoardGroupObs;
class Camera;
class Frame;

// CameraGroup is a group of Cameras which have common view of Board.
// For example, there are two boards in the scene, one of which is in the front
// while the other one in the back. And there are 6 regular cameras, three of
// them points to the front, and the other three of them point to the back. We
// say there are two CameraGroups in the scene.
class CameraGroup
{
public:
    CameraGroup() = delete;
    CameraGroup(int refCamId, int camGroupId);

    void insertFrame(std::shared_ptr<Frame> frame);
    void insertCamera(std::shared_ptr<Camera> camera);
    void insertBoardGroupObservation(
        std::shared_ptr<BoardGroupObs> observation);

    void setCameraPose(cv::InputArray pose, int camId);
    void setCameraPose(cv::Mat rvec, cv::Mat tvec, int camId);
    void getCameraPose(cv::OutputArray rvec, cv::OutputArray tvec,
                       int camId) const;
    cv::Mat cameraPose(int camId) const;
    inline auto cameraOrientation(int camId) const
    {
        cv::Mat rvec, _;
        getCameraPose(rvec, _, camId);
        return rvec;
    }
    inline auto cameraPosition(int camId) const
    {
        cv::Mat _, tvec;
        getCameraPose(_, tvec, camId);
        return tvec;
    }

    void computeObjPoseInCameraGroup();
    void refineCameraGroup(int iterations);
    void refineCameraGroupAndBoardGroups(int iterations);
    void refineCameraGroupAndBoardGroupsAndIntrinsics(int iterations);

    // Debug
    // TODO: This method only prints reprojection error
    void calcRpeMean() const;

public:
    // Observation of the 3D object (2d points)
    std::map<int, std::weak_ptr<BoardGroupObs>> _boardGroupObservations;
    // Frames containing boards for this cameras
    std::map<int, std::weak_ptr<Frame>> _frames;
    // cameras in the camera group
    std::map<int, std::weak_ptr<Camera>> _cameras;

    // TODO: Not used???
    std::vector<int> _visBoardGroupIds;

    // Extrinsic
    // camera pose wrt. the ref. cam
    std::map<int, std::array<double, 6>> _camPoses;
    std::vector<int> _camIds;
    int _refCamId;

    int _id;
};

} // namespace tl::mcmb
