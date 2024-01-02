#include "io_camera.h"

#include <magic_enum/magic_enum.hpp>
#include <json/json.hpp>

#include <AxCamera/Camera>
#include <AxCamera/DivisionUndistortionCameraModel>
#include <AxCamera/DoubleSphereCameraModel>
#include <AxCamera/ExtendedUnifiedCameraModel>
#include <AxCamera/FisheyeCameraModel>
#include <AxCamera/FovCameraModel>
#include <AxCamera/OmnidirectionalCameraModel>
#include <AxCamera/OrthographicCameraModel>
#include <AxCamera/PinholeCameraModel>
#include <AxCamera/PinholeRadialTangentialCameraModel>

namespace thoht::io {

namespace {
constexpr char kOmnidirectionalTypeName_coc[]{"MEI"};
constexpr char kFisheyeTypeName_coc[]{"KANNALA_BRANDT"};
} // namespace

namespace key {

// Built-in types
constexpr char kCamera[]{"camera"};
constexpr char kCameraName[]{"camera_name"};
constexpr char kImageWidth[]{"image_width"};
constexpr char kImageHeight[]{"image_height"};
constexpr char kCameraIntrinsics[]{"camera_intrinsics"};
constexpr char kCameraIntrinsicsType[]{"camera_intrinsics_type"};
constexpr char kCameraParameters[]{"parameters"};
constexpr char kCameraPose[]{"camera_pose"};

// CamOdoCal (coc) types
constexpr char kCameraIntrinsicsType_coc[]{"model_type"};
constexpr char kMirrorParams[]{"mirror_parameters"};
constexpr char kDistortionParams[]{"distortion_parameters"};
constexpr char kProjectionParams[]{"projection_parameters"};
constexpr char kK1[]{"k1"};
constexpr char kK2[]{"k2"};
constexpr char kK3[]{"k3"};
constexpr char kK4[]{"k4"};
constexpr char kK5[]{"k5"};
constexpr char kP1[]{"p1"};
constexpr char kP2[]{"p2"};
constexpr char kMu[]{"mu"};
constexpr char kMv[]{"mv"};
constexpr char kU0[]{"u0"};
constexpr char kV0[]{"v0"};
constexpr char kXi[]{"xi"};
constexpr char kGamma1[]{"gamma1"};
constexpr char kGamma2[]{"gamma2"};

} // namespace key

YAML::Node toCamOdoCalYamlNode(const Camera& camera, const std::string& name)
{
    const auto intrinsics = camera.cameraIntrinsics();
    const auto type = intrinsics->type();

    YAML::Node root;
    root[key::kCameraIntrinsicsType_coc] = [type]() -> std::string {
        switch (type) {
            case CameraIntrinsics::Type::Fisheye:
                return kFisheyeTypeName_coc;
            case CameraIntrinsics::Type::Omnidirectional:
                return kOmnidirectionalTypeName_coc;
            default: // TODO: Other models are different too
                return std::string{magic_enum::enum_name(type)};
        }
    }();
    root[key::kCameraName] = name;
    root[key::kImageWidth] = camera.imageWidth();
    root[key::kImageHeight] = camera.imageHeight();

    // Parmaters
    switch (type) {
        case CameraIntrinsics::Type::Fisheye: {
            if (const auto fisheye =
                    dynamic_cast<FisheyeCameraModel*>(intrinsics.get())) {
                YAML::Node projNode;
                projNode[key::kK2] = fisheye->radialDistortion1();
                projNode[key::kK3] = fisheye->radialDistortion2();
                projNode[key::kK4] = fisheye->radialDistortion3();
                projNode[key::kK5] = fisheye->radialDistortion4();
                projNode[key::kMu] = fisheye->focalLengthX();
                projNode[key::kMv] = fisheye->focalLengthY();
                projNode[key::kU0] = fisheye->principalPointX();
                projNode[key::kV0] = fisheye->principalPointY();

                root[key::kProjectionParams] = projNode;
            }
        } break;
        case CameraIntrinsics::Type::Omnidirectional: {
            if (const auto omni = dynamic_cast<OmnidirectionalCameraModel*>(
                    intrinsics.get())) {
                YAML::Node mirrorNode;
                mirrorNode[key::kXi] = omni->mirrorDistortion();

                YAML::Node distNode;
                distNode[key::kK1] = omni->radialDistortion1();
                distNode[key::kK2] = omni->radialDistortion2();
                distNode[key::kP1] = omni->tangentialDistortion1();
                distNode[key::kP2] = omni->tangentialDistortion2();

                YAML::Node projNode;
                projNode[key::kGamma1] = omni->focalLengthX();
                projNode[key::kGamma2] = omni->focalLengthY();
                projNode[key::kU0] = omni->principalPointX();
                projNode[key::kV0] = omni->principalPointY();

                root[key::kMirrorParams] = mirrorNode;
                root[key::kDistortionParams] = distNode;
                root[key::kProjectionParams] = projNode;
            }
        } break;
        default: { // TODO: camodocal::Pinhole/PinholeFull?
            const auto* parameters = camera.intrinsics();
            for (int i{0}; i < intrinsics->numParameters(); ++i) {
                root[key::kCameraParameters].push_back(parameters[i]);
            }
        } break;
    }

    return root;
}

bool fromCamOdoCalYamlNode(const YAML::Node& node, Camera& camera)
{
    const auto typeNode = node[key::kCameraIntrinsicsType_coc];
    if (!typeNode) {
        return false;
    }

    const auto type = [&typeNode]() -> std::optional<CameraIntrinsics::Type> {
        const auto typeName = typeNode.as<std::string>();

        if (!typeName.compare(kFisheyeTypeName_coc)) {
            return CameraIntrinsics::Type::Fisheye;
        }
        if (!typeName.compare(kOmnidirectionalTypeName_coc)) {
            return CameraIntrinsics::Type::Omnidirectional;
        }

        return magic_enum::enum_cast<CameraIntrinsics::Type>(typeName);
    }();

    if (!type.has_value()) {
        return false;
    }

    const auto imgWidthNode = node[key::kImageWidth];
    const auto imgHeightNode = node[key::kImageHeight];
    if (!imgWidthNode || !imgHeightNode) {
        return false;
    }

    camera.setCameraIntrinsicsModel(type.value());
    camera.setImageSize(imgHeightNode.as<int>(), imgHeightNode.as<int>());

    // Parameters, Assume all the keys exist
    auto intrinsics = camera.cameraIntrinsics();
    switch (type.value()) {
        case CameraIntrinsics::Type::Fisheye: {
            const auto projNode = node[key::kProjectionParams];
            intrinsics->setParameter(FisheyeCameraModel::K1,
                                     projNode[key::kK2].as<double>());
            intrinsics->setParameter(FisheyeCameraModel::K2,
                                     projNode[key::kK3].as<double>());
            intrinsics->setParameter(FisheyeCameraModel::K3,
                                     projNode[key::kK4].as<double>());
            intrinsics->setParameter(FisheyeCameraModel::K4,
                                     projNode[key::kK5].as<double>());
            const auto fx = projNode[key::kMu].as<double>();
            const auto fy = projNode[key::kMv].as<double>();
            intrinsics->setParameter(FisheyeCameraModel::Fx, fx);
            intrinsics->setParameter(FisheyeCameraModel::YX, fy / fx);
            intrinsics->setParameter(FisheyeCameraModel::Cx,
                                     projNode[key::kU0].as<double>());
            intrinsics->setParameter(FisheyeCameraModel::Cy,
                                     projNode[key::kV0].as<double>());
        } break;
        case CameraIntrinsics::Type::Omnidirectional: {
            const auto mirrorNode = node[key::kMirrorParams];
            intrinsics->setParameter(OmnidirectionalCameraModel::Xi,
                                     mirrorNode[key::kXi].as<double>());

            const auto distNode = node[key::kDistortionParams];
            intrinsics->setParameter(OmnidirectionalCameraModel::K1,
                                     distNode[key::kK1].as<double>());
            intrinsics->setParameter(OmnidirectionalCameraModel::K2,
                                     distNode[key::kK2].as<double>());
            intrinsics->setParameter(OmnidirectionalCameraModel::P1,
                                     distNode[key::kP1].as<double>());
            intrinsics->setParameter(OmnidirectionalCameraModel::P2,
                                     distNode[key::kP2].as<double>());

            const auto projNode = node[key::kProjectionParams];
            const auto fx = projNode[key::kGamma1].as<double>();
            const auto fy = projNode[key::kGamma2].as<double>();
            intrinsics->setParameter(OmnidirectionalCameraModel::Fx, fx);
            intrinsics->setParameter(OmnidirectionalCameraModel::YX, fy / fx);
            intrinsics->setParameter(OmnidirectionalCameraModel::Cx,
                                     projNode[key::kU0].as<double>());
            intrinsics->setParameter(OmnidirectionalCameraModel::Cy,
                                     projNode[key::kV0].as<double>());
        } break;
        default: { // TODO: camodocal::Pinhole/PinholeFull?
            const auto paramsNode = node[key::kCameraParameters];
            if (paramsNode.size() != intrinsics->numParameters()) {
                return false;
            }

            for (size_t i{0}; i < paramsNode.size(); ++i) {
                intrinsics->setParameter(i, paramsNode.as<double>());
            }
        } break;
    }

    return true;
}

} // namespace thoht::io

namespace thoht {

void to_json(nlohmann::json& json, const Camera& camera)
{
    // TODO
}

void from_json(const nlohmann::json& json, Camera& camera)
{
    // TODO
}

} // namespace thoht

namespace YAML {

using CameraConverter = convert<thoht::Camera>;

Node CameraConverter::encode(const thoht::Camera& camera)
{
    const auto intrinsics = camera.cameraIntrinsics();
    const auto* paramters = camera.intrinsics();

    namespace key = thoht::io::key;

    YAML::Node intriNode;
    intriNode[key::kCameraIntrinsicsType] =
        std::string{magic_enum::enum_name(intrinsics->type())};
    for (int i{0}; i < intrinsics->numParameters(); ++i) {
        intriNode[key::kCameraParameters].push_back(paramters[i]);
    }

    YAML::Node node;
    node[key::kImageWidth] = camera.imageWidth();
    node[key::kImageHeight] = camera.imageHeight();
    node[key::kCameraIntrinsics] = intriNode;

    // TODO: Camera pose

    return node;
}

bool CameraConverter::decode(const Node& node, thoht::Camera& camera)
{
    namespace key = thoht::io::key;

    const auto imgWidthNode = node[key::kImageWidth];
    const auto imgHeightNode = node[key::kImageHeight];
    const auto intriNode = node[key::kCameraIntrinsics];
    if (!imgWidthNode || !imgHeightNode || !intriNode) {
        return false;
    }

    // Assume all the keys in intrinsics node are exist.
    const auto typeName =
        intriNode[key::kCameraIntrinsicsType].as<std::string>();
    const auto type =
        magic_enum::enum_cast<thoht::CameraIntrinsics::Type>(typeName);
    if (!type.has_value()) {
        return false;
    }

    camera.setCameraIntrinsicsModel(type.value());

    // Intrinsics
    auto intrinsics = camera.cameraIntrinsics();
    const auto paramsNode = intriNode[key::kCameraParameters];
    if (paramsNode.size() != intrinsics->numParameters()) {
        return false;
    }

    for (size_t i{0}; i < paramsNode.size(); ++i) {
        intrinsics->setParameter(i, paramsNode.as<double>());
    }

    // TODO: Camera pose

    return true;
}

} // namespace YAML
