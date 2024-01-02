#include "orthographic_camera_model.h"

#include <glog/logging.h>
#include <magic_enum/magic_enum.hpp>

#include <AxCamera/CameraMatrixUtils>

namespace thoht {

OrthographicCameraModel::OrthographicCameraModel() : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.);
    setAspectRatio(1.);
    setPrincipalPoint(0., 0.);
    setParameter(Skew, 0.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
}

CameraIntrinsics::Type OrthographicCameraModel::type() const
{
    return Type::Orthographic;
}

void OrthographicCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    CameraIntrinsics::setFromMetaData(meta);

    if (meta.skew.is_set) {
        setParameter(Skew, meta.skew.value[0]);
    }

    if (meta.radial_distortion.is_set) {
        setParameter(K1, meta.radial_distortion.value[0]);
        setParameter(K2, meta.radial_distortion.value[1]);
    }
}

CameraMetaData OrthographicCameraModel::toMetaData() const
{
    auto meta = CameraIntrinsics::toMetaData();
    meta.skew.is_set = true;
    meta.skew.value[0] = skew();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = radialDistortion1();
    meta.radial_distortion.value[1] = radialDistortion2();

    return meta;
}

int OrthographicCameraModel::numParameters() const { return IntrinsicsSize; }

void OrthographicCameraModel::setSkew(double skew) { parameters_[Skew] = skew; }

double OrthographicCameraModel::skew() const { return parameters_[Skew]; }

void OrthographicCameraModel::setRadialDistortion(double k1, double k2)
{
    parameters_[K1] = k1;
    parameters_[K2] = k2;
}

double OrthographicCameraModel::radialDistortion1() const
{
    return parameters_[K1];
}

double OrthographicCameraModel::radialDistortion2() const
{
    return parameters_[K2];
}

std::vector<int> OrthographicCameraModel::constantParameterIndices(
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
    }
    return indices;
}

void OrthographicCameraModel::calibrationMatrix(Eigen::Matrix3d& K) const
{
    intrinsicsToCalibrationMatrix(parameters_[Fx], parameters_[Skew],
                                  parameters_[YX], parameters_[Cx],
                                  parameters_[Cy], K);
}

void OrthographicCameraModel::print() const
{
    LOG(INFO) << toLogString() << "Skew: " << skew()
              << "\n"
                 "Radial distortion (k1, k2): "
              << k1() << ", " << k2();
}

} // namespace thoht
