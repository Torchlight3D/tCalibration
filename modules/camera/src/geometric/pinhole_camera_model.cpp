#include "pinhole_camera_model.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

PinholeCameraModel::PinholeCameraModel() : Parent()
{
    setParameter(Skew, 0.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
}

void PinholeCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (meta.skew.has_value()) {
        setParameter(Skew, meta.skew.value()[0]);
    }

    if (meta.radialDistortion.has_value()) {
        setParameter(K1, meta.radialDistortion.value()[0]);
        setParameter(K2, meta.radialDistortion.value()[1]);
    }
}

CameraMetaData PinholeCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew = {skew()};
    meta.radialDistortion = {k1(), k2(), 0., 0.};

    return meta;
}

void PinholeCameraModel::setSkew(double skew) { params_[Skew] = skew; }

double PinholeCameraModel::skew() const { return params_[Skew]; }

void PinholeCameraModel::setRadialDistortion(double k1, double k2)
{
    params_[K1] = k1;
    params_[K2] = k2;
}

double PinholeCameraModel::radialDistortion1() const { return params_[K1]; }

double PinholeCameraModel::radialDistortion2() const { return params_[K2]; }

std::vector<int> PinholeCameraModel::fixedParameterIndices(
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
    }
    return indices;
}

Eigen::Matrix3d PinholeCameraModel::calibrationMatrix() const
{
    return intrinsicsToCalibrationMatrix(fx(), params_[Skew], aspectRatio(),
                                         cx(), cy());
}

std::string PinholeCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
        << "\n"
           "Skew: "
        << skew()
        << "\n"
           "Radial distortion (k1, k2): "
        << k1() << ", " << k2();
    return oss.str();
}

} // namespace tl
