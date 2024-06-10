#include "cameraadaptor.h"

#include <magic_enum/magic_enum.hpp>

namespace tl {

std::unique_ptr<Camera> fromCocCamera(
    const camodocal::Camera::ConstPtr coc_camera)
{
    if (!coc_camera) {
        return nullptr;
    }

    std::vector<double> params;
    coc_camera->writeParameters(params);

    CameraMetaData meta;
    meta.imageSize = {coc_camera->imageWidth(), coc_camera->imageHeight()};

    switch (coc_camera->modelType()) {
        using Type = CameraIntrinsicsType;
        using cocType = camodocal::Camera::ModelType;

        case cocType::KANNALA_BRANDT: {
            auto camera = std::make_unique<Camera>(Type::Fisheye);
            meta.intrinsicType = magic_enum::enum_name(Type::Fisheye);
            meta.focalLength = {params[4]};
            meta.aspectRatio = {params[5] / params[4]};
            meta.principalPoint = {params[6], params[7]};
            meta.radialDistortion = {params[0], params[1], params[2],
                                     params[3]};
            camera->setFromMetaData(meta);
            return camera;
        }
        case cocType::MEI: {
            auto camera = std::make_unique<Camera>(Type::Omnidirectional);
            meta.intrinsicType = magic_enum::enum_name(Type::Omnidirectional);
            meta.focalLength = {params[5]};
            meta.aspectRatio = {params[6] / params[5]};
            meta.principalPoint = {params[7], params[8]};
            // k1, k2, xi, placeholder
            meta.radialDistortion = {params[1], params[2], params[0], 0.};
            meta.tangentialDistortion = {params[3], params[4]};
            camera->setFromMetaData(meta);
            return camera;
        }
        case cocType::PINHOLE:
            // TODO
            break;
        case cocType::PINHOLE_FULL:
            // TODO
            break;
        case cocType::SCARAMUZZA:
            // Not supported
            break;
        default:
            break;
    }

    return nullptr;
}

} // namespace tl
