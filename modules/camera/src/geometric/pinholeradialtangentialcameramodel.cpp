#include "pinholeradialtangentialcameramodel.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

PinholeRadialTangentialCameraModel::PinholeRadialTangentialCameraModel()
    : Parent()
{
    setParameter(Skew, 0.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
    setParameter(K3, 0.);
    setParameter(T1, 0.);
    setParameter(T2, 0.);
}

void PinholeRadialTangentialCameraModel::setFromMetaData(
    const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (meta.skew.has_value()) {
        setParameter(Skew, meta.skew.value()[0]);
    }

    if (meta.radialDistortion.has_value()) {
        setParameter(K1, meta.radialDistortion.value()[0]);
        setParameter(K2, meta.radialDistortion.value()[1]);
        setParameter(K3, meta.radialDistortion.value()[2]);
    }

    if (meta.tangentialDistortion.has_value()) {
        setParameter(T1, meta.tangentialDistortion.value()[0]);
        setParameter(T2, meta.tangentialDistortion.value()[1]);
    }
}

CameraMetaData PinholeRadialTangentialCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew = {skew()};
    meta.radialDistortion = {k1(), k2(), k3(), 0.};
    meta.tangentialDistortion = {t1(), t2()};

    return meta;
}

void PinholeRadialTangentialCameraModel::setSkew(double skew)
{
    params_[Skew] = skew;
}

double PinholeRadialTangentialCameraModel::skew() const
{
    return params_[Skew];
}

void PinholeRadialTangentialCameraModel::setRadialDistortion(double k1,
                                                             double k2,
                                                             double k3)
{
    params_[K1] = k1;
    params_[K2] = k2;
    params_[K3] = k3;
}

double PinholeRadialTangentialCameraModel::radialDistortion1() const
{
    return params_[K1];
}

double PinholeRadialTangentialCameraModel::radialDistortion2() const
{
    return params_[K2];
}

double PinholeRadialTangentialCameraModel::radialDistortion3() const
{
    return params_[K3];
}

void PinholeRadialTangentialCameraModel::setTangentialDistortion(double t1,
                                                                 double t2)
{
    params_[T1] = t1;
    params_[T2] = t2;
}

double PinholeRadialTangentialCameraModel::tangentialDistortion1() const
{
    return params_[T1];
}

double PinholeRadialTangentialCameraModel::tangentialDistortion2() const
{
    return params_[T2];
}

std::vector<int> PinholeRadialTangentialCameraModel::fixedParameterIndices(
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
        indices.emplace_back(K3);
    }
    if ((flags & OptimizeIntrinsicsType::TangentialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(T1);
        indices.emplace_back(T2);
    }

    return indices;
}

Eigen::Matrix3d PinholeRadialTangentialCameraModel::calibrationMatrix() const
{
    return intrinsicsToCalibrationMatrix(fx(), params_[Skew], aspectRatio(),
                                         cx(), cy());
}

std::string PinholeRadialTangentialCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
        << "\n"
           "Skew: "
        << skew()
        << "\n"
           "Radial distortion (k1, k2, k3): "
        << k1() << ", " << k2() << ", " << k3()
        << "\n"
           "Tangential distortion (t1, t2): "
        << t1() << ", " << t2();
    return oss.str();
}

} // namespace tl
