#include "double_sphere_camera_model.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

DoubleSphereCameraModel::DoubleSphereCameraModel() : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.);
    setAspectRatio(1.);
    setPrincipalPoint(0., 0.);
    setParameter(Skew, 0.);
    setParameter(Xi, 0.);
    setParameter(Alpha, 0.5);
}

void DoubleSphereCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    CameraIntrinsics::setFromMetaData(meta);

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
    auto meta = CameraIntrinsics::toMetaData();
    meta.skew.is_set = true;
    meta.skew.value[0] = skew();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = alpha();
    meta.radial_distortion.value[1] = xi();

    return meta;
}

int DoubleSphereCameraModel::numParameters() const { return IntrinsicsSize; }

void DoubleSphereCameraModel::setSkew(double skew) { parameters_[Skew] = skew; }

double DoubleSphereCameraModel::skew() const { return parameters_[Skew]; }

void DoubleSphereCameraModel::setDistortion(double alpha, double xi)
{
    parameters_[Alpha] = alpha;
    parameters_[Xi] = xi;
}

double DoubleSphereCameraModel::alpha() const { return parameters_[Alpha]; }

double DoubleSphereCameraModel::xi() const { return parameters_[Xi]; }

std::vector<int> DoubleSphereCameraModel::constantParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    // All parameters
    if (flags == OptimizeIntrinsicsType::All) {
        return {};
    }

    auto indices = CameraIntrinsics::constantParameterIndices(flags);
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
    return intrinsicsToCalibrationMatrix(parameters_[Fx], parameters_[Skew],
                                         parameters_[YX], parameters_[Cx],
                                         parameters_[Cy]);
}

bool DoubleSphereCameraModel::isValid() const
{
    return CameraIntrinsics::isValid();
}

std::string DoubleSphereCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << CameraIntrinsics::toLog()
        << "\n"
           "Skew: "
        << skew()
        << "\n"
           "Radial distortion (alpha, xi): "
        << alpha() << ", " << xi();
    return oss.str();
}

} // namespace tl
