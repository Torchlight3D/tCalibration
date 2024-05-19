#pragma once

#include "Camera.h"

namespace camodocal {

class CameraFactory
{
public:
    static std::shared_ptr<CameraFactory> instance();

    Camera::Ptr generateCamera(Camera::ModelType modelType,
                               const std::string &cameraName,
                               const cv::Size &imageSize) const;

    Camera::Ptr create(const cv::FileNode &node);
    Camera::Ptr generateCameraFromYamlFile(const std::string &filename);

private:
    CameraFactory();
};

} // namespace camodocal
