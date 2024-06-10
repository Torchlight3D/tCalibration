#include "double_sphere_camera_model.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

DoubleSphereCameraModel::DoubleSphereCameraModel() : Parent()
{
    setParameter(Skew, 0.);
    setParameter(Xi, 0.);
    setParameter(Alpha, 0.5);
}

void DoubleSphereCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (meta.skew.is_set) {
        setParameter(Skew, meta.skew.value[0]);
    }

    if (meta.radial_distortion.is_set) {
        setParameter(Alpha, meta.radial_distortion.value[0]);
        setParameter(Xi, meta.radial_distortion.value[1]);
    }
}

CameraMetaData DoubleSphereCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew.is_set = true;
    meta.skew.value[0] = skew();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = alpha();
    meta.radial_distortion.value[1] = xi();

    return meta;
}

void DoubleSphereCameraModel::setSkew(double skew) { params_[Skew] = skew; }

double DoubleSphereCameraModel::skew() const { return params_[Skew]; }

void DoubleSphereCameraModel::setDistortion(double alpha, double xi)
{
    params_[Alpha] = alpha;
    params_[Xi] = xi;
}

double DoubleSphereCameraModel::alpha() const { return params_[Alpha]; }

double DoubleSphereCameraModel::xi() const { return params_[Xi]; }

std::vector<int> DoubleSphereCameraModel::fixedParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    // All parameters
    if (flags == OptimizeIntrinsicsType::All) {
        return {};
    }

    auto indices = Parent::fixedParameterIndices(flags);
    if ((flags & OptimizeIntrinsicsType::Skew) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Skew);
    }
    if ((flags & OptimizeIntrinsicsType::RadialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Xi);
        indices.emplace_back(Alpha);
    }
    return indices;
}

Eigen::Matrix3d DoubleSphereCameraModel::calibrationMatrix() const
{
    return intrinsicsToCalibrationMatrix(fx(), params_[Skew], aspectRatio(),
                                         cx(), cy());
}

std::string DoubleSphereCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
        << "\n"
           "Skew: "
        << skew()
        << "\n"
           "Radial distortion (alpha, xi): "
        << alpha() << ", " << xi();
    return oss.str();
}

} // namespace tl
