#include "camera_intrinsics.h"

#include <glog/logging.h>
#include <magic_enum/magic_enum.hpp>

#include "division_undistortion_camera_model.h"
#include "double_sphere_camera_model.h"
#include "extended_unified_camera_model.h"
#include "fisheye_camera_model.h"
#include "fov_camera_model.h"
#include "omnidirectional_camera_model.h"
#include "orthographic_camera_model.h"
#include "pinhole_camera_model.h"
#include "pinhole_radial_tangential_camera_model.h"
#include "../util_camera_matrix.h"

namespace tl {

using Eigen::MatrixXf;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;

CameraIntrinsics::Ptr CameraIntrinsics::create(Type type)
{
    switch (type) {
        case Type::Pinhole:
            return std::make_shared<PinholeCameraModel>();
        case Type::PinholeRadialTangential:
            return std::make_shared<PinholeRadialTangentialCameraModel>();
        case Type::Fisheye:
            return std::make_shared<FisheyeCameraModel>();
        case Type::Fov:
            return std::make_shared<FOVCameraModel>();
        case Type::DivisionUndistortion:
            return std::make_shared<DivisionUndistortionCameraModel>();
        case Type::DoubleSphere:
            return std::make_shared<DoubleSphereCameraModel>();
        case Type::ExtendedUnified:
            return std::make_shared<ExtendedUnifiedCameraModel>();
        case Type::Omnidirectional:
            return std::make_shared<OmnidirectionalCameraModel>();
        case Type::Orthographic:
            return std::make_shared<OrthographicCameraModel>();
        default:
            LOG(FATAL) << "Unsupported Camera model.";
            break;
    }

    return nullptr;
}

void CameraIntrinsics::setFromMetaData(const CameraMetaData& meta)
{
    if (meta.focal_length.is_set) {
        setFocalLength(meta.focal_length.value[0]);
    }
    else if (meta.image_width != 0. && meta.image_height != 0.) {
        setFocalLength(1.2 * static_cast<double>(std::max(meta.image_width,
                                                          meta.image_height)));
    }

    if (meta.principal_point.is_set) {
        setPrincipalPoint(meta.principal_point.value[0],
                          meta.principal_point.value[1]);
    }
    else if (meta.image_width != 0. && meta.image_height != 0.) {
        setPrincipalPoint(meta.image_width / 2., meta.image_height / 2.);
    }

    if (meta.aspect_ratio.is_set) {
        setAspectRatio(meta.aspect_ratio.value[0]);
    }
}

CameraMetaData CameraIntrinsics::toMetaData() const
{
    CameraMetaData meta;
    meta.camera_intrinsics_model_type = magic_enum::enum_name(type());
    meta.focal_length.is_set = true;
    meta.focal_length.value[0] = focalLength();
    meta.principal_point.is_set = true;
    meta.principal_point.value[0] = principalPointX();
    meta.principal_point.value[1] = principalPointY();
    meta.aspect_ratio.is_set = true;
    meta.aspect_ratio.value[0] = aspectRatio();

    return meta;
}

CameraIntrinsics& CameraIntrinsics::operator=(const CameraIntrinsics& o)
{
    CHECK(type() == o.type())
        << "Cannot assign camera intrinsics model of type "
        << static_cast<int>(o.type()) << " to a camera model of type "
        << static_cast<int>(type());

    parameters_ = o.parameters_;

    return *this;
}

void CameraIntrinsics::setFocalLength(double fx)
{
    CHECK_GT(fx, 0.0)
        << "Invalid focal length value. Focal length must be greater than 0.0.";
    setParameter(IntrinsicsIndex::Fx, fx);
}

double CameraIntrinsics::focalLength() const
{
    return parameter(IntrinsicsIndex::Fx);
}

void CameraIntrinsics::setAspectRatio(double y_x)
{
    CHECK_GT(y_x, 0.0)
        << "Invalid aspect ratio. Aspect ratio must be greater than 0.0.";
    setParameter(IntrinsicsIndex::YX, y_x);
}

double CameraIntrinsics::aspectRatio() const
{
    return parameter(IntrinsicsIndex::YX);
}

void CameraIntrinsics::setPrincipalPoint(double cx, double cy)
{
    setParameter(IntrinsicsIndex::Cx, cx);
    setParameter(IntrinsicsIndex::Cy, cy);
}

double CameraIntrinsics::principalPointX() const
{
    return parameter(IntrinsicsIndex::Cx);
}

double CameraIntrinsics::principalPointY() const
{
    return parameter(IntrinsicsIndex::Cy);
}

void CameraIntrinsics::setParameter(int index, double value)
{
    DCHECK_GE(index, 0);
    DCHECK_LT(index, parameters_.size());
    parameters_[index] = value;
}

double CameraIntrinsics::parameter(int index) const
{
    DCHECK_GE(index, 0);
    DCHECK_LT(index, parameters_.size());
    return parameters_[index];
}

const double* CameraIntrinsics::parameters() const
{
    return parameters_.data();
}

double* CameraIntrinsics::rParameters() { return parameters_.data(); }

std::vector<double> CameraIntrinsics::vector() const { return parameters_; }

std::vector<int> CameraIntrinsics::constantParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    std::vector<int> indices;
    if (flags == OptimizeIntrinsicsType::All) {
        return indices;
    }

    if ((flags & OptimizeIntrinsicsType::FocalLength) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Fx);
    }
    if ((flags & OptimizeIntrinsicsType::AspectRatio) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(YX);
    }
    if ((flags & OptimizeIntrinsicsType::PrincipalPoint) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Cx);
        indices.emplace_back(Cy);
    }

    return indices;
}

void CameraIntrinsics::calibrationMatrix(Matrix3d& K) const
{
    intrinsicsToCalibrationMatrix(parameters_[Fx], 0., parameters_[YX],
                                  parameters_[Cx], parameters_[Cy], K);
}

void CameraIntrinsics::createUndistortionMap(MatrixXf& map1, MatrixXf& map2,
                                             const Eigen::Vector2i& imageSize,
                                             double scale)
{
    //
}

void CameraIntrinsics::createUndistortionRectifyMap(
    MatrixXf& map1, MatrixXf& map2, Matrix3d& newK,
    const Eigen::Vector2i& imageSize, const Matrix3d& R)
{
    if (!(imageSize.array() > 0).all()) {
        LOG(ERROR) << "Failed to create undistortion map: "
                      "Invalid image size.";
        return;
    }

    if (fx() == 1. || fy() == 1.) {
        LOG(ERROR) << "Failed to create undistortion map: "
                      "Invalid intrinsics parameters.";
        return;
    }

    const auto width = imageSize.x();
    const auto height = imageSize.y();

    MatrixXf mapX = MatrixXf::Zero(width, height);
    MatrixXf mapY = MatrixXf::Zero(width, height);

    const Matrix3d R_inv = R.inverse();

    // Assume no skew
    // clang-format off
    Matrix3d K_rect;
    if (cx() == -1. || cy() == -1.) {
        K_rect << fx(),   0.,  width / 2.,
                    0., fy(), height / 2.,
                    0.,   0.,          1.;
    }
    else {
        K_rect << fx(),   0., cx(),
                    0., fy(), cy(),
                    0.,   0.,   1.;
    }
    // clang-format on

    const Matrix3d K_rect_inv = K_rect.inverse();

    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const Vector3d xo{static_cast<double>(u), static_cast<double>(v),
                              1.};
            const Vector3d uo = R_inv * K_rect_inv * xo;
            const Vector2d p = spaceToImage(uo);

            mapX(v, u) = p(0);
            mapY(v, u) = p(1);
        }
    }
}

std::string CameraIntrinsics::toLogString() const
{
    std::ostringstream oss;
    oss << "Camera model type: " << magic_enum::enum_name(type())
        << "\n"
           "Focal length (pixel): "
        << fx() << ", " << fy()
        << "\n"
           "Aspect ratio (height/width): "
        << aspectRatio()
        << "\n"
           "Principal point: "
        << cx() << ", " << cy() << std::endl;

    return oss.str();
}

} // namespace tl
