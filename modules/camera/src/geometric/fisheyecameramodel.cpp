#include "fisheyecameramodel.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

FisheyeCameraModel::FisheyeCameraModel() : Parent()
{
    setParameter(Skew, 0.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
    setParameter(K3, 0.);
    setParameter(K4, 0.);
}

void FisheyeCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (!meta.focalLength.has_value() && meta.imageSize.has_value()) {
        constexpr auto kEpiracalScale{0.4};
        setFocalLength(kEpiracalScale *
                       std::max(meta.imageWidth(), meta.imageHeight()));
    }

    if (meta.skew.has_value()) {
        setParameter(Skew, meta.skew.value()[0]);
    }

    if (meta.radialDistortion.has_value()) {
        setParameter(K1, meta.radialDistortion.value()[0]);
        setParameter(K2, meta.radialDistortion.value()[1]);
        setParameter(K3, meta.radialDistortion.value()[2]);
        setParameter(K4, meta.radialDistortion.value()[3]);
    }
}

CameraMetaData FisheyeCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew = {skew()};
    meta.radialDistortion = {k1(), k2(), k3(), k4()};

    return meta;
}

void FisheyeCameraModel::setSkew(double skew) { params_[Skew] = skew; }

double FisheyeCameraModel::skew() const { return params_[Skew]; }

void FisheyeCameraModel::setRadialDistortion(double k1, double k2, double k3,
                                             double k4)
{
    params_[K1] = k1;
    params_[K2] = k2;
    params_[K3] = k3;
    params_[K4] = k4;
}

double FisheyeCameraModel::radialDistortion1() const { return params_[K1]; }

double FisheyeCameraModel::radialDistortion2() const { return params_[K2]; }

double FisheyeCameraModel::radialDistortion3() const { return params_[K3]; }

double FisheyeCameraModel::radialDistortion4() const { return params_[K4]; }

std::vector<int> FisheyeCameraModel::fixedParameterIndices(
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
        indices.emplace_back(K1);
        indices.emplace_back(K2);
        indices.emplace_back(K3);
        indices.emplace_back(K4);
    }
    return indices;
}

Eigen::Matrix3d FisheyeCameraModel::calibrationMatrix() const
{
    return intrinsicsToCalibrationMatrix(fx(), params_[Skew], aspectRatio(),
                                         cx(), cy());
}

std::string FisheyeCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
        << "\n"
           "Skew: "
        << skew()
        << "\n"
           "Radial distortion (k1, k2, k3, k4): "
        << k1() << ", " << k2() << ", " << k3() << ", " << k4();
    return oss.str();
}

} // namespace tl
