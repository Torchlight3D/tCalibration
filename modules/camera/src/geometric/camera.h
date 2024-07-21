#pragma once

#include <tMath/Eigen/Types>

#include "cameraintrinsics.h"

namespace tl {

class Camera
{
public:
    explicit Camera(CameraIntrinsicsType type = CameraIntrinsicsType::Pinhole);
    // TODO: Duplicated code, write a swap
    Camera(const Camera& other);
    Camera& operator=(const Camera& other);
    ~Camera();

    void deepCopy(const Camera& other);

    void setFromMetaData(const CameraMetaData& meta);
    CameraMetaData toMetaData() const;

    bool setFromProjectMatrix(int imageWidth, int imageHeight,
                              const Matrix34d& projectionMatrix);

    /// ----------------------- Properties ----------------------
    ///
    void setImageSize(int width, int height);
    int imageWidth() const;
    int imageHeight() const;

    void setCalibrated(bool on);
    bool calibrated() const;

    /// ----------------------- Intrinsics ----------------------
    ///
    void setCameraIntrinsicsModel(CameraIntrinsicsType type);
    CameraIntrinsicsType cameraIntrinsicsModel() const;

    void setFocalLength(double fx);
    double focalLength() const;
    // TODO: What is the physical meaning? learn from Colmap
    double meanFocalLength() const;

    void setPrincipalPoint(double cx, double cy);
    double principalPointX() const;
    double principalPointY() const;
    inline Eigen::Vector2d principalPoint() const
    {
        return {principalPointX(), principalPointY()};
    }

    void setCameraIntrinsics(CameraIntrinsics::Ptr intrinsics);
    CameraIntrinsics::Ptr cameraIntrinsics() const;
    const double* intrinsics() const;
    double* rIntrinsics();
    std::vector<double> parameters() const;

    Eigen::Matrix3d calibrationMatrix() const;

    /// ----------------------- Extrinsics ------------------------
    ///
    void setPosition(const Eigen::Vector3d& position);
    Eigen::Vector3d position() const;

    void setOrientationFromRotationMatrix(const Eigen::Matrix3d& rotation);
    void setOrientationFromAngleAxis(const Eigen::Vector3d& rvec);
    Eigen::Matrix3d orientationAsRotationMatrix() const;
    Eigen::Vector3d orientationAsAngleAxis() const;
    Eigen::Vector3d orientationAsEuler() const;

    enum ExtrinsicIndex
    {
        Position = 0,
        Orientation = 3,

        ExtrinsicsSize = 6,
    };
    const double* extrinsics() const;
    double* rExtrinsics();

    /// ---------------------- Goemetry ---------------------------
    ///
    // From world to image
    void projectionMatrix(Matrix34d& matrix) const;
    // From image to world
    void invProjectionMatrix(Matrix34d& matrix) const;

    Eigen::Vector3d projectionCenter() const;

    /// -------------------- Point Mapping ------------------------
    ///
    // Projects a homogeneous 3D point into the image plane and undistorts the
    // point according to the distortion parameters. The function returns
    // the depth of the point so that points that project behind the camera
    // (i.e., negative depth) can be determined. Points at infinity return a
    // depth of infinity.
    double projectPoint(const Eigen::Vector4d& point, Eigen::Vector2d& pixel,
                        const Eigen::Vector3d* orientation = nullptr,
                        const Eigen::Vector3d* translation = nullptr) const;

    // Converts the pixel point to a ray in 3D space such that the origin of
    // the ray is at the camera center and the direction is the pixel
    // direction rotated according to the camera orientation in 3D space.
    //
    // NOTE: The depth of the ray is set to 1. so that we can remain consistent
    // with projectPoint(). That is:
    // if we have:
    //     d = projectPoint(X, &x)
    //     r = PixelToUnitDepthRay(x)
    // then it will be the case that
    //     X = c + r * d
    // where X is the 3D point
    //       x is the image projection of X
    //       c is the camera position
    //       r is the ray obtained from PixelToRay
    //       d is the depth of the 3D point with respect to the image
    Eigen::Vector3d pixelToUnitDepthRay(const Eigen::Vector2d& pixel) const;

    // Converts image pixel coordinates to normalized coordinates in the camera
    // coordinates by removing the effect of camera intrinsics/calibration. This
    // method is similar to pixelToUnitDepthRay except that it only removes the
    // effect of camera calibration and does not account for the camera pose.
    Eigen::Vector3d pixelToNormalizedCoordinates(
        const Eigen::Vector2d& pixel) const;

    void transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                   double scale = 1.);

    /// Debugs

private:
    double extrinsics_[ExtrinsicsSize];
    CameraIntrinsics::Ptr intrinsics_;
    int img_size_[2]; // width, height
    bool calibrated_;
};

std::ostream& operator<<(std::ostream& os, const Camera& cam);

} // namespace tl
