#include "orthographiccameramodel.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

OrthographicCameraModel::OrthographicCameraModel() : Parent()
{
    setParameter(Skew, 0.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
}

void OrthographicCameraModel::setFromMetaData(const CameraMetaData& meta)
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

CameraMetaData OrthographicCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew = {skew()};
    meta.radialDistortion = {k1(), k2(), 0., 0.};

    return meta;
}

void OrthographicCameraModel::setSkew(double skew) { params_[Skew] = skew; }

double OrthographicCameraModel::skew() const { return params_[Skew]; }

void OrthographicCameraModel::setRadialDistortion(double k1, double k2)
{
    params_[K1] = k1;
    params_[K2] = k2;
}

double OrthographicCameraModel::radialDistortion1() const
{
    return params_[K1];
}

double OrthographicCameraModel::radialDistortion2() const
{
    return params_[K2];
}

std::vector<int> OrthographicCameraModel::fixedParameterIndices(
    OptimizeIntrinsicsType flags) const
{
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

Eigen::Matrix3d OrthographicCameraModel::calibrationMatrix() const
{
    return intrinsicsToCalibrationMatrix(fx(), params_[Skew], aspectRatio(),
                                         cx(), cy());
}

std::string OrthographicCameraModel::toLog() const
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
