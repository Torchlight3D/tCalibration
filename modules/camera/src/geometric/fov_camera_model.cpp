#include "fov_camera_model.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

FovCameraModel::FovCameraModel() : Parent()
{
    setParameter(Omega, kDefaultOmega);
}

void FovCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (meta.radialDistortion.has_value()) {
        setParameter(Omega, meta.radialDistortion.value()[0]);
    }
    else {
        setParameter(Omega, kDefaultOmega);
    }
}

CameraMetaData FovCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.radialDistortion = {omega(), 0., 0., 0.};

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
