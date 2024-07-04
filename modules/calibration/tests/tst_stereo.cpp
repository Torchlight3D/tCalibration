#include <filesystem>
#include <format>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include <tCalibration/CalibrationIO>
#include <tCalibration/CameraIntrinsicsCalibration>
#include <tCalibration/StereoCameraCalibration>
#include <tCalibration/StereoCameraVerification>

namespace fs = std::filesystem;
using namespace tl;

TEST(Stereo, Intrinsics)
{
    //
    GTEST_SKIP() << "Not ready yet...";
}

TEST(Stereo, Extrinsics)
{
    //
    GTEST_SKIP() << "Not ready yet...";
}

TEST(Stereo, Overall)
{
    GTEST_SKIP() << "Skip stereo calibration test case.";

    StereoCameraCalibration stereoCalib;
    EXPECT_TRUE(true);
}

TEST(Stereo, VerifyVioDataIo)
{
    const auto currentPath = fs::current_path();
    const auto savedDataDir = currentPath / "data" / "verification" / "good";

    EXPECT_TRUE(fs::exists(savedDataDir))
        << "SavedData directory doesn't exist.";

    const auto verifyFilename =
        std::format("{}-{}", "verification", io::kCalibParameterFilename);
    for (const auto& dataDir : fs::directory_iterator(savedDataDir)) {
        const auto filename = (dataDir.path() / verifyFilename).string();

        StereoCameraVerification::VerifyData data;
        EXPECT_TRUE(
            StereoCameraVerification::VerifyData::loadFromFile(filename, data))
            << "Failed to load calibration file: " << filename;
    }
}

TEST(Stereo, VioVerificationGood)
{
    const auto currentPath = fs::current_path();
    const auto savedDataDir = currentPath / "data" / "verification" / "good";

    EXPECT_TRUE(fs::exists(savedDataDir))
        << "SavedData directory doesn't exist.";

    using Error = StereoCameraVerification::Summary::Error;

    StereoCameraVerification::Options opts;
    opts.chessPatternSize = {11, 8};
    opts.chessSize = {0.045, 0.045};
    opts.reference = []() {
        StereoCameraVerification::Reference ref;
        ref.maxTrackerRejectRate = 0.1;
        ref.maxEstimatorRejectRate = 0.1;
        ref.maxStereoOptimizationCost = 20.;
        ref.maxRejectRelativeSizeMedian = 6e-3;
        ref.maxRejectRelativePositionMedian = 6e-3;
        return ref;
    }();

    const char leftName[]{"verify_left-0.png"};
    const char rightName[]{"verify_right-0.png"};
    const auto verifyFilename =
        std::format("{}-{}", "verification", io::kCalibParameterFilename);
    for (const auto& dataDir : fs::directory_iterator(savedDataDir)) {
        const auto filename = (dataDir.path() / verifyFilename).string();
        const auto leftFilename = (dataDir.path() / leftName).string();
        const auto rightFilename = (dataDir.path() / rightName).string();
        const auto left = cv::imread(leftFilename);
        const auto right = cv::imread(rightFilename);

        StereoCameraVerification::VerifyData data;
        EXPECT_TRUE(
            StereoCameraVerification::VerifyData::loadFromFile(filename, data))
            << "Failed to load calibration file: " << filename;

        StereoCameraVerification verification{data, opts};
        const auto summary = verification.startVerify(left, right);
        EXPECT_TRUE(summary.error == Error::NoError)
            << "Verification error: " << summary.error;
    }
}

TEST(Stereo, VioVerificationBad)
{
    GTEST_SKIP() << "Not ready yet...";
    // TODO
}
