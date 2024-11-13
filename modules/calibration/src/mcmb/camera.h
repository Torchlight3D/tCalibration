#pragma once

#include <map>

#include <opencv2/core/mat.hpp>

namespace tl::mcmb {

class BoardGroupObs;
class BoardObs;
class Frame;

// The camera in the system. Contains:
// + OpenCV style intrinsics
// + Frames captured
// + Boards observed
// + BoardGroup observed
class Camera
{
public:
    Camera() = delete;
    Camera(int cameraId, int type);

    void insertFrame(std::shared_ptr<Frame> frame);
    void insertBoardObservation(std::shared_ptr<BoardObs> observation);
    void insertBoardGroupObservation(
        std::shared_ptr<BoardGroupObs> observation);
    void clear();

    void setCameraMatrix(const cv::Mat K);
    void setDistortion(const cv::Mat dist);
    inline void setIntrinsics(const cv::Mat K, const cv::Mat dist)
    {
        setCameraMatrix(K);
        setDistortion(dist);
    }
    cv::Mat cameraMatrix() const;
    cv::Mat distortion() const;
    inline void getIntrinsics(cv::Mat &cameraMatrix, cv::Mat &distortion) const
    {
        cameraMatrix = this->cameraMatrix();
        distortion = this->distortion();
    }

    void initializeCalibration();
    void refineIntrinsicCalibration(int iterations);

public:
    // TODO: These are {Index -> Observation}, can't see any reason to use map.
    // Observation of the boards (2d points)
    std::map<int, std::weak_ptr<BoardObs>> _boardObservations;
    // Observation of the 3D object (2d points)
    std::map<int, std::weak_ptr<BoardGroupObs>> _boardGroupObservations;

    // TODO: These are Ids ??
    // vector of index of the 3D boards
    std::vector<int> _visBoardIds;
    // vector of index of the 3D object
    std::vector<int> _visBoardGroupIds;

    // intrinsics
    // fx,fy,u0,v0,r1,r2,t1,t2,r3 (perspective)
    // fx,fy,u0,v0,k1,k2,k3,k4 (Kannala)
    std::array<double, 9> intrinsics_;

    int _id = 0;
    int _type = 0;
    cv::Size _imgSize;

private:
    bool checkPointsCloseToBorder(std::shared_ptr<BoardObs> boardObs) const;

private:
    // Frames containing boards for this cameras
    std::map<int, std::weak_ptr<Frame>> _frames;
};

} // namespace tl::mcmb
