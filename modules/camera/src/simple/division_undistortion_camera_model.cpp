#include "division_undistortion_camera_model.h"

#include <glog/logging.h>
#include <magic_enum/magic_enum.hpp>

#include <tCamera/CameraMatrixUtils>

namespace tl {

namespace {
constexpr double kDefaultRadialDistortion{0.};
}

DivisionUndistortionCameraModel::DivisionUndistortionCameraModel()
    : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.);
    setAspectRatio(1.);
    setPrincipalPoint(0., 0.);
    setParameter(K, kDefaultRadialDistortion);
}

CameraIntrinsics::Type DivisionUndistortionCameraModel::type() const
{
    return Type::DivisionUndistortion;
}

void DivisionUndistortionCameraModel::setFromMetaData(
    const CameraMetaData& meta)
{
    CameraIntrinsics::setFromMetaData(meta);

    if (meta.radial_distortion.is_set) {
        setParameter(K, meta.radial_distortion.value[0]);
    }
    else {
        setParameter(K, kDefaultRadialDistortion);
    }
}

CameraMetaData DivisionUndistortionCameraModel::toMetaData() const
{
    auto meta = CameraIntrinsics::toMetaData();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = radialDistortion1();

    return meta;
}

int DivisionUndistortionCameraModel::numParameters() const
{
    return IntrinsicsSize;
}

void DivisionUndistortionCameraModel::setRadialDistortion(double k)
{
    parameters_[K] = k;
}

double DivisionUndistortionCameraModel::radialDistortion1() const
{
    return parameters_[K];
}

std::vector<int> DivisionUndistortionCameraModel::constantParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    // All parameters
    if (flags == OptimizeIntrinsicsType::All) {
        return {};
    }

    auto indices = CameraIntrinsics::constantParameterIndices(flags);
    if ((flags & OptimizeIntrinsicsType::RadialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(K);
    }

    return indices;
}

void DivisionUndistortionCameraModel::print() const
{
    LOG(INFO) << toLogString() << "Radial distortion (k): " << k();
}

} // namespace tl
