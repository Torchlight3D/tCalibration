#include "omnidirectional_camera_model.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

OmnidirectionalCameraModel::OmnidirectionalCameraModel() : CameraIntrinsics()
{
    parameters_.resize(IntrinsicsSize);
    setFocalLength(1.);
    setAspectRatio(1.);
    setPrincipalPoint(0., 0.);
    setParameter(Xi, 1.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
    setParameter(P1, 0.);
    setParameter(P2, 0.);
}

void OmnidirectionalCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    CameraIntrinsics::setFromMetaData(meta);

    if (meta.radial_distortion.is_set) {
        setParameter(K1, meta.radial_distortion.value[0]);
        setParameter(K2, meta.radial_distortion.value[1]);
        setParameter(Xi, meta.radial_distortion.value[2]);
    }

    if (meta.tangential_distortion.is_set) {
        setParameter(P1, meta.tangential_distortion.value[0]);
        setParameter(P2, meta.tangential_distortion.value[1]);
    }
}

CameraMetaData OmnidirectionalCameraModel::toMetaData() const
{
    auto meta = CameraIntrinsics::toMetaData();
    meta.radial_distortion.is_set = true;
    meta.radial_distortion.value[0] = radialDistortion1();
    meta.radial_distortion.value[1] = radialDistortion2();
    // Put mirror parameter in radial distortion
    meta.radial_distortion.value[2] = mirrorDistortion();
    meta.tangential_distortion.is_set = true;
    meta.tangential_distortion.value[0] = tangentialDistortion1();
    meta.tangential_distortion.value[1] = tangentialDistortion2();

    return meta;
}

int OmnidirectionalCameraModel::numParameters() const { return IntrinsicsSize; }

void OmnidirectionalCameraModel::setMirrorDistortion(double xi)
{
    parameters_[Xi] = xi;
}

double OmnidirectionalCameraModel::mirrorDistortion() const
{
    return parameters_[Xi];
}

void OmnidirectionalCameraModel::setRadialDistortion(double k1, double k2)
{
    parameters_[K1] = k1;
    parameters_[K2] = k2;
}

double OmnidirectionalCameraModel::radialDistortion1() const
{
    return parameters_[K1];
}

double OmnidirectionalCameraModel::radialDistortion2() const
{
    return parameters_[K2];
}

void OmnidirectionalCameraModel::setTangentialDistortion(double p1, double p2)
{
    parameters_[P1] = p1;
    parameters_[P2] = p2;
}

double OmnidirectionalCameraModel::tangentialDistortion1() const
{
    return parameters_[P1];
}

double OmnidirectionalCameraModel::tangentialDistortion2() const
{
    return parameters_[P2];
}

std::vector<int> OmnidirectionalCameraModel::constantParameterIndices(
    OptimizeIntrinsicsType flags) const
{
    if (flags == OptimizeIntrinsicsType::All) {
        return {};
    }

    std::vector<int> indices;
    if ((flags & OptimizeIntrinsicsType::FocalLength) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Fx);
        // NOTE: Xi is related focal length
        indices.emplace_back(Xi);
    }
    if ((flags & OptimizeIntrinsicsType::AspectRatio) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(YX);
    }
    if ((flags & OptimizeIntrinsicsType::PrincipalPoint) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(Cx);
        indices.emplace_back(Cy);
    }

    if ((flags & OptimizeIntrinsicsType::RadialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(K1);
        indices.emplace_back(K2);
    }
    if ((flags & OptimizeIntrinsicsType::TangentialDistortion) ==
        OptimizeIntrinsicsType::None) {
        indices.emplace_back(P1);
        indices.emplace_back(P2);
    }

    return indices;
}

bool OmnidirectionalCameraModel::isValid() const
{
    return CameraIntrinsics::isValid() && (xi() < 2. && xi() > 1.4);
}

std::string OmnidirectionalCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << CameraIntrinsics::toLog()
        << "\n"
           "Mirror distortion (xi): "
        << xi()
        << "\n"
           "Radial distortion (k1, k2): "
        << k1() << ", " << k2()
        << "\n"
           "Tangential distortion (p1, p2): "
        << p1() << ", " << p2();
    return oss.str();
}

} // namespace tl
