#include "fov_camera_model.h"

#include <glog/logging.h>
#include <magic_enum/magic_enum.hpp>

#include <tCamera/CameraMatrixUtils>

namespace tl {

namespace {
constexpr double kDefaultOmega{0.75};
}

FOVCameraModel::FOVCameraModel() : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.0);
    setAspectRatio(1.0);
    setPrincipalPoint(0.0, 0.0);
    setParameter(Omega, kDefaultOmega);
}

CameraIntrinsics::Type FOVCameraModel::type() const { return Type::Fov; }

void FOVCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    // NOTE: Don't call parent method on purpose
    if (meta.focal_length.is_set) {
        setFocalLength(meta.focal_length.value[0]);
    }
    else if (meta.image_width != 0.0 && meta.image_height != 0.0) {
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
        setParameter(YX, meta.aspect_ratio.value[0]);
    }

    if (meta.radial_distortion.is_set) {
        setParameter(Omega, meta.radial_distortion.value[0]);
    }
    else {
        setParameter(Omega, kDefaultOmega);
    }
}

CameraMetaData FOVCameraModel::toMetaData() const
{
    auto meta = CameraIntrinsics::toMetaData();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = radialDistortion1();

    return meta;
}

int FOVCameraModel::numParameters() const { return IntrinsicsSize; }

void FOVCameraModel::setRadialDistortion(double omega)
{
    parameters_[Omega] = omega;
}

double FOVCameraModel::radialDistortion1() const { return parameters_[Omega]; }

std::vector<int> FOVCameraModel::constantParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    // All parameters
    if (flags == OptimizeIntrinsicsType::All) {
        return {};
    }

    auto indices = CameraIntrinsics::constantParameterIndices(flags);
    if ((flags & OptimizeIntrinsicsType::RadialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Omega);
    }
    return indices;
}

void FOVCameraModel::print() const
{
    LOG(INFO) << toLogString() << "Radial distortion (omega): " << omega();
}

} // namespace tl
