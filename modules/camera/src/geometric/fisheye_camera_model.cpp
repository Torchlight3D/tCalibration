#include "fisheye_camera_model.h"

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
    if (meta.focal_length.is_set) {
        setFocalLength(meta.focal_length.value[0]);
    }
    else if (meta.image_width != 0. && meta.image_height != 0.) {
        // NOTE: A scaling of 0.4 times the max image dimension was empircally
        // observed to be a decent initialization.
        constexpr double kFocalLengthScaleFactor{0.4};
        setFocalLength(
            kFocalLengthScaleFactor *
            static_cast<double>(std::max(meta.image_width, meta.image_height)));
    }

    if (meta.principal_point.is_set) {
        setPrincipalPoint(meta.principal_point.value[0],
                          meta.principal_point.value[1]);
    }
    else if (meta.image_width != 0. && meta.image_height != 0.) {
        setPrincipalPoint(meta.image_width / 2., meta.image_height / 2.);
    }

    if (meta.aspect_ratio.is_set) {
        setParameter(YX, meta.aspect_ratio.value[0]);
    }

    if (meta.skew.is_set) {
        setParameter(Skew, meta.skew.value[0]);
    }

    if (meta.radial_distortion.is_set) {
        setParameter(K1, meta.radial_distortion.value[0]);
        setParameter(K2, meta.radial_distortion.value[1]);
        setParameter(K3, meta.radial_distortion.value[2]);
        setParameter(K4, meta.radial_distortion.value[3]);
    }
}

CameraMetaData FisheyeCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.skew.is_set = true;
    meta.skew.value[0] = skew();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = radialDistortion1();
    meta.radial_distortion.value[1] = radialDistortion2();
    meta.radial_distortion.value[2] = radialDistortion3();
    meta.radial_distortion.value[3] = radialDistortion4();

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
