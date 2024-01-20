#include "pinhole_radial_tangential_camera_model.h"

#include <glog/logging.h>
#include <magic_enum/magic_enum.hpp>

#include <tCamera/CameraMatrixUtils>

namespace tl {

PinholeRadialTangentialCameraModel::PinholeRadialTangentialCameraModel()
    : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.);
    setAspectRatio(1.);
    setPrincipalPoint(0., 0.);
    setParameter(Skew, 0.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
    setParameter(K3, 0.);
    setParameter(T1, 0.);
    setParameter(T2, 0.);
}

CameraIntrinsics::Type PinholeRadialTangentialCameraModel::type() const
{
    return Type::PinholeRadialTangential;
}

void PinholeRadialTangentialCameraModel::setFromMetaData(
    const CameraMetaData& meta)
{
    CameraIntrinsics::setFromMetaData(meta);

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
    auto meta = CameraIntrinsics::toMetaData();
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

int PinholeRadialTangentialCameraModel::numParameters() const
{
    return IntrinsicsSize;
}

void PinholeRadialTangentialCameraModel::setSkew(double skew)
{
    parameters_[Skew] = skew;
}

double PinholeRadialTangentialCameraModel::skew() const
{
    return parameters_[Skew];
}

void PinholeRadialTangentialCameraModel::setRadialDistortion(double k1,
                                                             double k2,
                                                             double k3)
{
    parameters_[K1] = k1;
    parameters_[K2] = k2;
    parameters_[K3] = k3;
}

double PinholeRadialTangentialCameraModel::radialDistortion1() const
{
    return parameters_[K1];
}

double PinholeRadialTangentialCameraModel::radialDistortion2() const
{
    return parameters_[K2];
}

double PinholeRadialTangentialCameraModel::radialDistortion3() const
{
    return parameters_[K3];
}

void PinholeRadialTangentialCameraModel::setTangentialDistortion(double t1,
                                                                 double t2)
{
    parameters_[T1] = t1;
    parameters_[T2] = t2;
}

double PinholeRadialTangentialCameraModel::tangentialDistortion1() const
{
    return parameters_[T1];
}

double PinholeRadialTangentialCameraModel::tangentialDistortion2() const
{
    return parameters_[T2];
}

std::vector<int> PinholeRadialTangentialCameraModel::constantParameterIndices(
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

void PinholeRadialTangentialCameraModel::calibrationMatrix(
    Eigen::Matrix3d& K) const
{
    intrinsicsToCalibrationMatrix(parameters_[Fx], parameters_[Skew],
                                  parameters_[YX], parameters_[Cx],
                                  parameters_[Cy], K);
}

void PinholeRadialTangentialCameraModel::print() const
{
    LOG(INFO) << toLogString() << "Skew: " << skew()
              << "\n"
                 "Radial distortion (k1, k2, k3): "
              << k1() << ", " << k2() << ", " << k3()
              << "\n"
                 "Tangential distortion (t1, t2): "
              << t1() << ", " << t2();
}

} // namespace tl
