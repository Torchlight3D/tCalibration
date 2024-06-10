#include "cameraadaptor.h"

#include <magic_enum/magic_enum.hpp>

namespace tl {

Camera fromCocCamera(const camodocal::Camera::ConstPtr coc_camera)
{
    Camera camera;
    if (!coc_camera) {
        return camera;
    }

    std::vector<double> params;
    coc_camera->writeParameters(params);

    CameraMetaData meta;
    meta.image_width = coc_camera->imageWidth();
    meta.image_height = coc_camera->imageHeight();

    switch (coc_camera->modelType()) {
        using Type = CameraIntrinsicsType;
        using cocType = camodocal::Camera::ModelType;

        case cocType::KANNALA_BRANDT: {
            meta.camera_intrinsics_model_type =
                magic_enum::enum_name(Type::Fisheye);
            meta.focal_length.value[0] = params[4];
            meta.focal_length.is_set = true;
            meta.aspect_ratio.value[0] = params[5] / params[4];
            meta.aspect_ratio.is_set = true;
            meta.principal_point.value[0] = params[6];
            meta.principal_point.value[1] = params[7];
            meta.principal_point.is_set = true;
            meta.radial_distortion.value[0] = params[0];
            meta.radial_distortion.value[1] = params[1];
            meta.radial_distortion.value[2] = params[2];
            meta.radial_distortion.value[3] = params[3];
            meta.radial_distortion.is_set = true;
            camera.setFromMetaData(meta);
        } break;
        case cocType::MEI: {
            meta.camera_intrinsics_model_type =
                magic_enum::enum_name(Type::Omnidirectional);
            meta.focal_length.value[0] = params[5];
            meta.focal_length.is_set = true;
            meta.aspect_ratio.value[0] = params[6] / params[5];
            meta.aspect_ratio.is_set = true;
            meta.principal_point.value[0] = params[7];
            meta.principal_point.value[1] = params[8];
            meta.principal_point.is_set = true;
            meta.radial_distortion.value[0] = params[1];
            meta.radial_distortion.value[1] = params[2];
            meta.radial_distortion.value[2] = params[0]; // Xi
            meta.radial_distortion.is_set = true;
            meta.tangential_distortion.value[0] = params[3];
            meta.tangential_distortion.value[1] = params[4];
            meta.radial_distortion.is_set = true;
            camera.setFromMetaData(meta);
        } break;
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

    return camera;
}

} // namespace tl
