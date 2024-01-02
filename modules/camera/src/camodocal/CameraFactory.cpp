#include "CameraFactory.h"

#include <iostream>

#include "CataCamera.h"
#include "EquidistantCamera.h"
#include "PinholeCamera.h"
#include "PinholeFullCamera.h"
#include "ScaramuzzaCamera.h"

namespace camodocal {

std::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory() {}

std::shared_ptr<CameraFactory> CameraFactory::instance()
{
    if (!m_instance.get()) {
        m_instance.reset(new CameraFactory);
    }

    return m_instance;
}

Camera::Ptr CameraFactory::generateCamera(Camera::ModelType modelType,
                                          const std::string &cameraName,
                                          const cv::Size &imageSize) const
{
    switch (modelType) {
        case Camera::KANNALA_BRANDT: {
            EquidistantCamera::Ptr camera(new EquidistantCamera);

            EquidistantCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        case Camera::PINHOLE: {
            PinholeCamera::Ptr camera(new PinholeCamera);

            PinholeCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        case Camera::PINHOLE_FULL: {
            PinholeFullCamera::Ptr camera(new PinholeFullCamera);

            PinholeFullCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        case Camera::SCARAMUZZA: {
            OCAMCamera::Ptr camera(new OCAMCamera);

            OCAMCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        case Camera::MEI:
        default: {
            CataCamera::Ptr camera(new CataCamera);

            CataCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
    }
}

Camera::Ptr CameraFactory::create(const cv::FileNode &node)
{
    // TODO: Use template
    const auto camerasmodel = node[Camera::Parameters::keyModelType].string();
    if (!camerasmodel.compare(CataCamera::Parameters::kModelTypeName)) {
        CataCamera::Parameters params;
        if (!params.read(node)) {
            return nullptr;
        }

        return std::make_shared<CataCamera>(params);
    }
    if (!camerasmodel.compare(PinholeCamera::Parameters::kModelTypeName)) {
        PinholeCamera::Parameters params;
        if (!params.read(node)) {
            return nullptr;
        }

        return std::make_shared<PinholeCamera>(params);
    }
    if (!camerasmodel.compare(PinholeFullCamera::Parameters::kModelTypeName)) {
        PinholeFullCamera::Parameters params;
        if (!params.read(node)) {
            return nullptr;
        }

        return std::make_shared<PinholeFullCamera>(params);
    }
    if (!camerasmodel.compare(EquidistantCamera::Parameters::kModelTypeName)) {
        EquidistantCamera::Parameters params;
        if (!params.read(node)) {
            return nullptr;
        }

        return std::make_shared<EquidistantCamera>(params);
    }
    if (!camerasmodel.compare(OCAMCamera::Parameters::kModelTypeName)) {
        OCAMCamera::Parameters params;
        if (!params.read(node)) {
            return nullptr;
        }

        return std::make_shared<OCAMCamera>(params);
    }

    std::cerr << "ERROR: the camera model is not supported." << std::endl;
    return nullptr;
}

Camera::Ptr CameraFactory::generateCameraFromYamlFile(
    const std::string &filename)
{
    cv::FileStorage fs{filename, cv::FileStorage::READ};
    if (!fs.isOpened()) {
        return nullptr;
    }

    Camera::ModelType modelType = Camera::MEI;
    const auto modelTypeNode = fs[Camera::Parameters::keyModelType];
    if (modelTypeNode.isNone()) {
        std::cerr << "ERROR: Empty camera model type." << std::endl;
        return nullptr;
    }

    std::string modelTypeName;
    modelTypeNode >> modelTypeName;
    if (!modelTypeName.compare(EquidistantCamera::Parameters::kModelTypeName)) {
        modelType = Camera::KANNALA_BRANDT;
    }
    else if (!modelTypeName.compare(CataCamera::Parameters::kModelTypeName)) {
        modelType = Camera::MEI;
    }
    else if (!modelTypeName.compare(OCAMCamera::Parameters::kModelTypeName)) {
        modelType = Camera::SCARAMUZZA;
    }
    else if (!modelTypeName.compare(
                 PinholeCamera::Parameters::kModelTypeName)) {
        modelType = Camera::PINHOLE;
    }
    else if (!modelTypeName.compare(
                 PinholeFullCamera::Parameters::kModelTypeName)) {
        modelType = Camera::PINHOLE_FULL;
    }
    else {
        std::cerr << "ERROR: Unknown camera model: " << modelTypeName
                  << std::endl;
        return nullptr;
    }

    switch (modelType) {
        case Camera::KANNALA_BRANDT: {
            EquidistantCamera::Ptr camera(new EquidistantCamera);

            EquidistantCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        case Camera::PINHOLE: {
            PinholeCamera::Ptr camera(new PinholeCamera);

            PinholeCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        case Camera::PINHOLE_FULL: {
            PinholeFullCamera::Ptr camera(new PinholeFullCamera);

            PinholeFullCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        case Camera::SCARAMUZZA: {
            OCAMCamera::Ptr camera(new OCAMCamera);

            OCAMCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        case Camera::MEI:
        default: {
            CataCamera::Ptr camera(new CataCamera);

            CataCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
    }

    return nullptr;
}
} // namespace camodocal
