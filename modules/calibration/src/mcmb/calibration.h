#pragma once

#include <Eigen/Geometry>
#include <nlohmann_json/json.hpp>
#include <opencv2/core/mat.hpp>

#include <tVision/Target/AprilTag>
#include <tCore/Global>

namespace tl::mcmb {

class Camera;

class Calibration
{
public:
    struct Options
    {
        /////// Cameras

        // Image size of all cameras. Now we assume all the cameras share same
        // image size, but it's possible for current system to handle different
        // image sizes.
        cv::Size imageSize{};

        // Camera intrinsic model used in calibration.
        // OpenCV supported types: 0. Pinhole (Brown); 1. Fisheye (KB)
        std::vector<int> intrinsicModels;

        /////// Boards

        // Boards' configurations in the scene.
        // Boards with different layouts and tag size are supported, require:
        // 1. Tag ids on different boards should be identical, and in ascending
        // order.
        // 2. Tags' family should be consistent.
        std::vector<AprilTag::Board::Options> boardOptions;

        double minDetectRateInBoard{0.3};

        /////// Algorithm

        // Optimization threshold (in pixel) in Ransac PnP
        float errorThreshold{10.f};

        // Number of iteration in Ransac PnP
        int numIterations{1000};

        // Fix camera intrinsic parameters during optimization
        bool fixIntrinsics{false};

        bool useQuaternionAverage{true};

        Options() {}

        bool isValid() const;
    };

    explicit Calibration(const Options &options = {});
    ~Calibration();

    /// Properties
    const Options &options() const;
    void setOptions(const Options &options);
    bool setFromJson(const std::string &json);
    void setupCameras(const std::vector<int> &intrinsicModels,
                      const cv::Size &imageSize);
    void setupBoards(const std::vector<AprilTag::Board::Options> &options);

    /// Data
    // Incremental interface
    void addBoardDetections(
        int frameId, int camId,
        const std::vector<AprilTag::Board::Detection> &detections,
        double timestamp = 0., const std::string &filename = {});
    void clearData();

    /// Actions
    void startCalibration();

    /// Debug
    void saveResults(const std::string &taskDir) const;
    std::pair<double, double> samplePeriod() const;
    // FIXME: These two methods assume the camera group is a stereo camera.
    void getExtrinsics(Eigen::Matrix4d &transform) const;
    bool getCameraPoses(int camId, std::vector<double> &stamps,
                        std::vector<Eigen::Vector3d> &translations,
                        std::vector<Eigen::Quaterniond> &rotations) const;

    const Camera *const camera(int index) const;

private:
    DISABLE_COPY(Calibration);
    class Impl;
    const std::unique_ptr<Impl> d;
};

} // namespace tl::mcmb

namespace nlohmann {

void to_json(json &j, const tl::mcmb::Calibration::Options &opts);
void from_json(const json &j, tl::mcmb::Calibration::Options &opts);

} // namespace nlohmann
