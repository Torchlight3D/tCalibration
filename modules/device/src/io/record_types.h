#pragma once

namespace cv {
class Mat;
} // namespace cv

namespace thoht {

struct Vector3d
{
    double x, y, z;
};

struct Quaternion
{
    double x, y, z, w;
};

struct Pose
{
    // Timestamp in seconds.
    double time;

    // 3D position in a right-handed metric coordinate system where the z-axis
    // points up.
    Vector3d position;

    // Orientation quaternion in the same coordinate system as position
    Quaternion orientation;
};

struct FrameData
{
    double t;

    double focalLengthX;
    double focalLengthY;
    double px;
    double py;

    int cameraId;

    // Optional: Frame data as an cv::Mat. If present, recorded to a video file.
    const cv::Mat *frameData = nullptr;
};

struct AccelerometerData
{
    double t;
    double x, y, z;
    double temperature = -1.0;
};

struct GyroscopeData
{
    double t;
    double x, y, z;
    double temperature = -1.0;
};

} // namespace thoht
