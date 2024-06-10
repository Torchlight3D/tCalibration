﻿#include "pinhole_radial_tangential_camera_model.h"

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

    if (meta.skew.is_set) {
        setParameter(Skew, meta.skew.value[0]);
    }

    if (meta.radial_distortion.is_set) {
        setParameter(K1, meta.radial_distortion.value[0]);
        setParameter(K2, meta.radial_distortion.value[1]);
        setParameter(K3, meta.radial_distortion.value[2]);
    }

    if (meta.tangential_distortion.is_set) {
        setParameter(T1, meta.tangential_distortion.value[0]);
        setParameter(T2, meta.tangential_distortion.value[1]);
    }
}

CameraMetaData PinholeRadialTangentialCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew.is_set = true;
    meta.skew.value[0] = skew();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = radialDistortion1();
    meta.radial_distortion.value[1] = radialDistortion2();
    meta.radial_distortion.value[2] = radialDistortion3();
    meta.tangential_distortion.is_set = true;
    meta.tangential_distortion.value[0] = tangentialDistortion1();
    meta.tangential_distortion.value[1] = tangentialDistortion2();

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
