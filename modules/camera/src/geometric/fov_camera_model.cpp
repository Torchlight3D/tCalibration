#include "fov_camera_model.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

FovCameraModel::FovCameraModel() : Parent()
{
    setParameter(Omega, kDefaultOmega);
}

void FovCameraModel::setFromMetaData(const CameraMetaData& meta)
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

CameraMetaData FovCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = radialDistortion1();

    return meta;
}

void FovCameraModel::setRadialDistortion(double omega)
{
    params_[Omega] = omega;
}

double FovCameraModel::radialDistortion1() const { return params_[Omega]; }

std::vector<int> FovCameraModel::fixedParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    // All parameters
    if (flags == OptimizeIntrinsicsType::All) {
        return {};
    }

    auto indices = Parent::fixedParameterIndices(flags);
    if ((flags & OptimizeIntrinsicsType::RadialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Omega);
    }
    return indices;
}

std::string FovCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
        << "\n"
           "Radial distortion (omega): "
        << omega();
    return oss.str();
}

} // namespace tl
