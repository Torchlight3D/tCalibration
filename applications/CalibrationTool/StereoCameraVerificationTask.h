#pragma once

#include <opencv2/core/mat.hpp>

#include <json/json.hpp>

#include <tCore/EnumUtils>
#include <tCore/Global>

namespace tl {

class StereoCameraVerifyScene;

class StereoCameraVerificationTask
{
public:
    // TODO: Use move on data
    StereoCameraVerificationTask(
        const StereoCameraCalibResult &data,
        std::unique_ptr<StereoCameraVerifyScene> scene);
    ~StereoCameraVerificationTask();

    struct Reference
    {
        // Image quality
        double minSharpness = 0.6;

        // Tracker
        verify::TrackerReference tracker = {};

        // Estimator
        verify::EstimatorReference estimator = {};

        // Reprojections
        verify::ReprojectionReference reprojection = {};

        // Stereo rectify
        double maxStereoMatchingDiffY = 0.3;
    };

    // TODO:
    // 1. Add ItemStatus
    // 2. Distinguish ItemStatus and Error
    // 3. Try to separate raw item values and item status
    struct Summary
    {
        enum Item
        {
            /// Image quality
            Sharpness,

            /// VIO tracker
            EpipolarErrorMax,
            EpipolarErrorMean,

            /// VIO estimator
            RelativeSizeFailureRate,
            RelativeSizeDiffMedian,
            ShortDistanceRelativeSizeDiffIqr,

            /// Reprojections
            RightToLeftRpeMedian,

            /// Stereo rectify
            StereoRectifyDiffYMean,

            Count,
        };

        std::array<double, 2> sharpness;
        std::vector<double> epipolarErrorMax = {};
        std::vector<double> epipolarErrorMean = {};
        std::vector<double> relativeSizeFailureRate = {};
        std::vector<double> relativeSizeDiffMedian = {};
        double shortDistanceRelativeSizeDiffIqr = 0.;
        std::vector<double> rightToLeftRpeMedian = {};
        double stereoRectifyDiffYMean = {};

        enum Error
        {
            None = 0x0000,
            InvalidConfigs = 0x0001,

            /// Data
            InvalidImage = 0x0002,
            FailedDetectTarget = 0x0004,

            /// Image quality
            BlurryImage = 0x0008,

            /// VIO tracker
            LargeEpipolarErrorMax = 0x0010,
            LargeEpipolarErrorMean = 0x0020,

            /// VIO estimator
            PoorRelativeSizeEsitmation = 0x0040,
            LargeRelativeSizeDiffMedian = 0x0080,
            LargeShortDistanceRelativeSizeDiffIqr = 0x0100,

            /// Reprojections
            LargeRightToLeftRpeMedian = 0x0200,

            /// Stereo
            LargeStereoMatchingDiffY = 0x0400,
        };
        Error errors{Error::None};

        bool passed() const;
    };

    Summary verify(cv::InputArray left, cv::InputArray right,
                   const Reference &ref, cv::OutputArray viz = cv::noArray());

    static Summary::Error notAppliedErrors();

private:
    DISABLE_COPY(StereoCameraVerificationTask)

    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

MAKE_FLAGS(StereoCameraVerificationTask::Summary::Error)

// std::iostream interface
std::ostream &operator<<(std::ostream &os,
                         const StereoCameraVerificationTask::Summary &summary);

} // namespace tl

namespace nlohmann {

void to_json(json &j, const tl::StereoCameraVerificationTask::Reference &ref);
void from_json(const json &j, tl::StereoCameraVerificationTask::Reference &ref);

void to_json(json &j, const tl::StereoCameraVerificationTask::Summary &summary);
void from_json(const json &j,
               tl::StereoCameraVerificationTask::Summary &summary);

} // namespace nlohmann
