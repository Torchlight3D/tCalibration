#include "divisionundistortioncameramodel.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

DivisionUndistortionCameraModel::DivisionUndistortionCameraModel() : Parent()
{
    setParameter(K, 0.);
}

void DivisionUndistortionCameraModel::setFromMetaData(
    const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (meta.radialDistortion.has_value()) {
        setParameter(K, meta.radialDistortion.value()[0]);
    }
    else {
        setParameter(K, 0.);
    }
}

CameraMetaData DivisionUndistortionCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.radialDistortion = {k(), 0., 0., 0.};

    return meta;
}

void DivisionUndistortionCameraModel::setRadialDistortion(double k)
{
    params_[K] = k;
}

double DivisionUndistortionCameraModel::radialDistortion1() const
{
    return params_[K];
}

std::vector<int> DivisionUndistortionCameraModel::fixedParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    // All parameters
    if (flags == OptimizeIntrinsicsType::All) {
        return {};
    }

    auto indices = Parent::fixedParameterIndices(flags);
    if ((flags & OptimizeIntrinsicsType::RadialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(K);
    }

    return indices;
}

std::string DivisionUndistortionCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
        << "\n"
           "Radial distortion (k): "
        << k();
    return oss.str();
}

} // namespace tl
