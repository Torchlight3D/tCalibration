#include "extended_unified_camera_model.h"

#include <glog/logging.h>
#include <magic_enum/magic_enum.hpp>

#include <AxCamera/CameraMatrixUtils>

namespace thoht {

ExtendedUnifiedCameraModel::ExtendedUnifiedCameraModel() : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.);
    setAspectRatio(1.);
    setPrincipalPoint(0., 0.);
    setParameter(Skew, 0.);
    setParameter(Alpha, 0.5);
    setParameter(Beta, 0.);
}

CameraIntrinsics::Type ExtendedUnifiedCameraModel::type() const
{
    return Type::ExtendedUnified;
}

void ExtendedUnifiedCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    CameraIntrinsics::setFromMetaData(meta);

    if (meta.skew.is_set) {
        setParameter(Skew, meta.skew.value[0]);
    }

    if (meta.radial_distortion.is_set) {
        setParameter(Alpha, meta.radial_distortion.value[0]);
        setParameter(Beta, meta.radial_distortion.value[1]);
    }
}

CameraMetaData ExtendedUnifiedCameraModel::toMetaData() const
{
    auto meta = CameraIntrinsics::toMetaData();
    meta.skew.is_set = true;
    meta.skew.value[0] = skew();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = alpha();
    meta.radial_distortion.value[1] = beta();

    return meta;
}

int ExtendedUnifiedCameraModel::numParameters() const { return IntrinsicsSize; }

void ExtendedUnifiedCameraModel::setSkew(double skew)
{
    parameters_[Skew] = skew;
}

double ExtendedUnifiedCameraModel::skew() const { return parameters_[Skew]; }

void ExtendedUnifiedCameraModel::setDistortion(double alpha, double beta)
{
    parameters_[Alpha] = alpha;
    parameters_[Beta] = beta;
}

double ExtendedUnifiedCameraModel::alpha() const { return parameters_[Alpha]; }

double ExtendedUnifiedCameraModel::beta() const { return parameters_[Beta]; }

std::vector<int> ExtendedUnifiedCameraModel::constantParameterIndices(
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
        indices.emplace_back(Alpha);
        indices.emplace_back(Beta);
    }
    return indices;
}

void ExtendedUnifiedCameraModel::calibrationMatrix(Eigen::Matrix3d& K) const
{
    intrinsicsToCalibrationMatrix(parameters_[Fx], parameters_[Skew],
                                  parameters_[YX], parameters_[Cx],
                                  parameters_[Cy], K);
}

void ExtendedUnifiedCameraModel::print() const
{
    LOG(INFO) << toLogString() << "Skew: " << skew()
              << "\n"
                 "Radial distortion (alpha, beta): "
              << alpha() << ", " << beta();
}

} // namespace thoht
