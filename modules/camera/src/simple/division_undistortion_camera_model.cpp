#include "division_undistortion_camera_model.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

DivisionUndistortionCameraModel::DivisionUndistortionCameraModel()
    : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.);
    setAspectRatio(1.);
    setPrincipalPoint(0., 0.);
    setParameter(K, 0.);
}

void DivisionUndistortionCameraModel::setFromMetaData(
    const CameraMetaData& meta)
{
    CameraIntrinsics::setFromMetaData(meta);

    if (meta.radial_distortion.is_set) {
        setParameter(K, meta.radial_distortion.value[0]);
    }
    else {
        setParameter(K, 0.);
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

bool DivisionUndistortionCameraModel::isValid() const
{
    return CameraIntrinsics::isValid();
}

std::string DivisionUndistortionCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << CameraIntrinsics::toLog()
        << "\n"
           "Radial distortion (k): "
        << k();
    return oss.str();
}

} // namespace tl
