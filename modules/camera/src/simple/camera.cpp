#include "camera.h"

#include <algorithm>

#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <magic_enum/magic_enum.hpp>

#include <tCamera/CameraMatrixUtils>
#include <tCore/Math>
#include <tMath/Eigen/Utils>

#include "pinhole_camera_model.h"

namespace tl {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
inline Eigen::Vector3d projectionCenterFromProjectionMatrix(const Matrix34d& P)
{
    return -P.leftCols<3>().transpose() * P.rightCols<1>();
}
} // namespace

Camera::Camera(CameraIntrinsics::Type type) : calibrated_(false)
{
    Map<Eigen::Matrix<double, 1, ExtrinsicsSize>>(rExtrinsics()).setZero();
    intrinsics_ = CameraIntrinsics::create(type);

    img_size_[0] = 0;
    img_size_[1] = 0;
}

Camera::Camera(const Camera& o)
{
    // Copy the extrinsics.
    std::copy(o.extrinsics(), o.extrinsics() + ExtrinsicsSize, extrinsics_);
    // Perform a shallow copy of the camera intrinsics.
    intrinsics_ = o.intrinsics_;
    img_size_[0] = o.img_size_[0];
    img_size_[1] = o.img_size_[1];
    calibrated_ = o.calibrated_;
}

Camera& Camera::operator=(const Camera& o)
{
    // Copy the extrinsics.
    std::copy(o.extrinsics(), o.extrinsics() + ExtrinsicsSize, extrinsics_);
    // Perform a shallow copy of the camera intrinsics.
    intrinsics_ = o.intrinsics_;
    img_size_[0] = o.img_size_[0];
    img_size_[1] = o.img_size_[1];
    calibrated_ = o.calibrated_;
    return *this;
}

Camera::~Camera() = default;

void Camera::deepCopy(const Camera& camera)
{
    // extrinsics
    std::copy(camera.extrinsics(), camera.extrinsics() + ExtrinsicsSize,
              extrinsics_);

    // intrinsics
    intrinsics_ = CameraIntrinsics::create(camera.cameraIntrinsicsModel());
    const auto& other = *camera.cameraIntrinsics();
    std::copy(other.parameters(), other.parameters() + other.numParameters(),
              intrinsics_->rParameters());

    // size
    img_size_[0] = camera.img_size_[0];
    img_size_[1] = camera.img_size_[1];
}

void Camera::setFromMetaData(const CameraMetaData& meta)
{
    const auto intriType = magic_enum::enum_cast<CameraIntrinsics::Type>(
                               meta.camera_intrinsics_model_type)
                               .value_or(CameraIntrinsics::Type::Pinhole);
    if (intrinsics_->type() != intriType) {
        intrinsics_ = CameraIntrinsics::create(intriType);
    }

    img_size_[0] = meta.image_width;
    img_size_[1] = meta.image_height;
    intrinsics_->setFromMetaData(meta);
}

CameraMetaData Camera::toMetaData() const
{
    CameraMetaData meta = intrinsics_->toMetaData();
    meta.image_width = img_size_[0];
    meta.image_height = img_size_[1];
    return meta;
}

bool Camera::setFromProjectMatrix(int imageWidth, int imageHeight,
                                  const Matrix34d& P)
{
    DCHECK_GT(imageWidth, 0);
    DCHECK_GT(imageHeight, 0);
    img_size_[0] = imageWidth;
    img_size_[1] = imageHeight;

    intrinsics_ = std::make_shared<PinholeCameraModel>();

    Matrix3d K;
    Vector3d rvec, tvec;
    decomposeProjectionMatrix(P, K, rvec, tvec);

    Map<Vector3d>(rExtrinsics() + Orientation) = rvec;
    Map<Vector3d>(rExtrinsics() + Position) = tvec;

    using math::isApprox0;

    if (isApprox0(K(0, 0)) || isApprox0(K(1, 1))) {
        LOG(INFO) << "Cannot set focal lengths to zero!";
        return false;
    }

    double* params = intrinsics_->rParameters();
    calibrationMatrixToIntrinsics(
        K, params + PinholeCameraModel::Fx, params + PinholeCameraModel::Skew,
        params + PinholeCameraModel::YX, params + PinholeCameraModel::Cx,
        params + PinholeCameraModel::Cy);
    return true;
}

void Camera::setImageSize(int width, int height)
{
    img_size_[0] = width;
    img_size_[1] = height;
}

int Camera::imageWidth() const { return img_size_[0]; }

int Camera::imageHeight() const { return img_size_[1]; }

void Camera::setCalibrated(bool on) { calibrated_ = on; }

bool Camera::calibrated() const { return calibrated_; }

void Camera::setCameraIntrinsicsModel(CameraIntrinsics::Type type)
{
    if (cameraIntrinsicsModel() != type) {
        intrinsics_ = CameraIntrinsics::create(type);
    }
}

CameraIntrinsics::Type Camera::cameraIntrinsicsModel() const
{
    return intrinsics_->type();
}

void Camera::setFocalLength(double fx) { intrinsics_->setFocalLength(fx); }

double Camera::focalLength() const { return intrinsics_->focalLength(); }

double Camera::meanFocalLength() const
{
    return intrinsics_->focalLength() * (1 + intrinsics_->aspectRatio()) * 0.5;
}

void Camera::setPrincipalPoint(double cx, double cy)
{
    intrinsics_->setPrincipalPoint(cx, cy);
}

double Camera::principalPointX() const
{
    return intrinsics_->principalPointX();
}

double Camera::principalPointY() const
{
    return intrinsics_->principalPointY();
}

void Camera::setCameraIntrinsics(CameraIntrinsics::Ptr intrinsics)
{
    intrinsics_ = intrinsics;
}

CameraIntrinsics::Ptr Camera::cameraIntrinsics() const { return intrinsics_; }

const double* Camera::intrinsics() const { return intrinsics_->parameters(); }

double* Camera::rIntrinsics() { return intrinsics_->rParameters(); }

std::vector<double> Camera::parameters() const { return intrinsics_->vector(); }

void Camera::calibrationMatrix(Matrix3d& K) const
{
    intrinsics_->calibrationMatrix(K);
}

void Camera::setPosition(const Vector3d& position)
{
    Map<Vector3d>(rExtrinsics() + Position) = position;
}

Vector3d Camera::position() const
{
    return Map<const Vector3d>(extrinsics() + Position);
}

void Camera::setOrientationFromRotationMatrix(const Matrix3d& rotation)
{
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(rotation.data()),
        rExtrinsics() + Orientation);
}

void Camera::setOrientationFromAngleAxis(const Vector3d& rvec)
{
    Map<Vector3d>(rExtrinsics() + Orientation) = rvec;
}

Matrix3d Camera::orientationAsRotationMatrix() const
{
    Matrix3d rotation;
    ceres::AngleAxisToRotationMatrix(
        extrinsics() + Orientation,
        ceres::ColumnMajorAdapter3x3(rotation.data()));
    return rotation;
}

Vector3d Camera::orientationAsAngleAxis() const
{
    return Map<const Vector3d>(extrinsics() + Orientation);
}

Vector3d Camera::orientationAsEuler() const
{
    double roll, pitch, yaw;
    math::RotationMatrixToRPY(orientationAsRotationMatrix(), roll, pitch, yaw);
    return {roll, pitch, yaw};
}

const double* Camera::extrinsics() const { return extrinsics_; }

double* Camera::rExtrinsics() { return extrinsics_; }

void Camera::projectionMatrix(Matrix34d& P) const
{
    Matrix3d K;
    intrinsics_->calibrationMatrix(K);
    composeProjectionMatrix(K, orientationAsAngleAxis(), position(), P);
}

void Camera::invProjectionMatrix(Matrix34d& matrix) const
{
    Matrix34d P;
    projectionMatrix(P);

    matrix.leftCols<3>() = P.leftCols<3>().transpose();
    matrix.rightCols<1>() = projectionCenterFromProjectionMatrix(P);
}

Vector3d Camera::projectionCenter() const
{
    const auto quat = Quaterniond{orientationAsRotationMatrix()}.normalized();
    const auto tvec = position();
    return quat * -tvec;
}

double Camera::projectPoint(const Vector4d& point, Vector2d& pixel,
                            const Vector3d* rvec, const Vector3d* tvec) const
{
    // FIXME: Feel like something wrong here
    const auto rotation = (rvec ? (*rvec) : orientationAsAngleAxis());
    const auto translation = (tvec ? (*tvec) : position());

    const Vector3d adjusted_point = point.head<3>() - point[3] * translation;
    Vector3d rotated_point;
    ceres::AngleAxisRotatePoint(rotation.data(), adjusted_point.data(),
                                rotated_point.data());
    pixel = intrinsics_->spaceToImage(rotated_point);

    return rotated_point[2] / point[3];
}

Vector3d Camera::pixelToUnitDepthRay(const Vector2d& pixel) const
{
    // Remove the effect of calibration.
    const Vector3d undistorted_point = pixelToNormalizedCoordinates(pixel);

    // Apply rotation.
    const auto rotation = orientationAsRotationMatrix();
    const Vector3d direction = rotation.transpose() * undistorted_point;
    return direction;
}

Vector3d Camera::pixelToNormalizedCoordinates(const Vector2d& pixel) const
{
    return intrinsics_->imageToSpace(pixel);
}

void Camera::print() const
{
    intrinsics_->print();
    LOG(INFO) << "Camera pose in world coordinate: "
                 "\n"
                 "Position: "
              << position().transpose()
              << "\n"
                 "Orientation (roll, pitch, yaw):"
              << orientationAsEuler().transpose();
}

} // namespace tl
