#include "omnidirectionalcameramodel.h"

#include <tCamera/CameraMatrixUtils>

namespace tl {

OmnidirectionalCameraModel::OmnidirectionalCameraModel() : Parent()
{
    setParameter(Xi, 1.);
    setParameter(K1, 0.);
    setParameter(K2, 0.);
    setParameter(P1, 0.);
    setParameter(P2, 0.);
}

void OmnidirectionalCameraModel::setFromMetaData(const CameraMetaData& meta)
{
    Parent::setFromMetaData(meta);

    if (meta.radialDistortion.has_value()) {
        setParameter(K1, meta.radialDistortion.value()[0]);
        setParameter(K2, meta.radialDistortion.value()[1]);
        setParameter(Xi, meta.radialDistortion.value()[2]);
    }

    if (meta.tangentialDistortion.has_value()) {
        setParameter(P1, meta.tangentialDistortion.value()[0]);
        setParameter(P2, meta.tangentialDistortion.value()[1]);
    }
}

CameraMetaData OmnidirectionalCameraModel::toMetaData() const
{
    auto meta = Parent::toMetaData();
    meta.radialDistortion = {k1(), k2(), xi(), 0.};
    meta.tangentialDistortion = {p1(), p2()};

    return meta;
}

void OmnidirectionalCameraModel::setMirrorDistortion(double xi)
{
    params_[Xi] = xi;
}

double OmnidirectionalCameraModel::mirrorDistortion() const
{
    return params_[Xi];
}

void OmnidirectionalCameraModel::setRadialDistortion(double k1, double k2)
{
    params_[K1] = k1;
    params_[K2] = k2;
}

double OmnidirectionalCameraModel::radialDistortion1() const
{
    return params_[K1];
}

double OmnidirectionalCameraModel::radialDistortion2() const
{
    return params_[K2];
}

void OmnidirectionalCameraModel::setTangentialDistortion(double p1, double p2)
{
    params_[P1] = p1;
    params_[P2] = p2;
}

double OmnidirectionalCameraModel::tangentialDistortion1() const
{
    return params_[P1];
}

double OmnidirectionalCameraModel::tangentialDistortion2() const
{
    return params_[P2];
}

std::vector<int> OmnidirectionalCameraModel::fixedParameterIndices(
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

std::string OmnidirectionalCameraModel::toLog() const
{
    std::ostringstream oss;
    oss << Parent::toLog()
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
