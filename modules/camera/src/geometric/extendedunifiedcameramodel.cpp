#include "extendedunifiedcameramodel.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

ExtendedUnifiedCameraModel::ExtendedUnifiedCameraModel() : Parent()
{
    setParameter(Skew, 0.);
    setParameter(Alpha, 0.5);
    setParameter(Beta, 0.);
}

void ExtendedUnifiedCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (meta.skew.has_value()) {
        setParameter(Skew, meta.skew.value()[0]);
    }

    if (meta.radialDistortion.has_value()) {
        setParameter(Alpha, meta.radialDistortion.value()[0]);
        setParameter(Beta, meta.radialDistortion.value()[1]);
    }
}

CameraMetaData ExtendedUnifiedCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew = {skew()};
    meta.radialDistortion = {alpha(), beta(), 0., 0.};

    return meta;
}

void ExtendedUnifiedCameraModel::setSkew(double skew) { params_[Skew] = skew; }

double ExtendedUnifiedCameraModel::skew() const { return params_[Skew]; }

void ExtendedUnifiedCameraModel::setDistortion(double alpha, double beta)
{
    params_[Alpha] = alpha;
    params_[Beta] = beta;
}

double ExtendedUnifiedCameraModel::alpha() const { return params_[Alpha]; }

double ExtendedUnifiedCameraModel::beta() const { return params_[Beta]; }

std::vector<int> ExtendedUnifiedCameraModel::fixedParameterIndices(
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
        indices.emplace_back(Alpha);
        indices.emplace_back(Beta);
    }
    return indices;
}

Eigen::Matrix3d ExtendedUnifiedCameraModel::calibrationMatrix() const
{
    return intrinsicsToCalibrationMatrix(fx(), params_[Skew], aspectRatio(),
                                         cx(), cy());
}

std::string ExtendedUnifiedCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
        << "\n"
           "Skew: "
        << skew()
        << "\n"
           "Radial distortion (alpha, beta): "
        << alpha() << ", " << beta();
    return oss.str();
}

} // namespace tl
