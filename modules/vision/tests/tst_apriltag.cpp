#include <filesystem>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include <tVision/Target/KalibrAprilTagBoard>

namespace fs = std::filesystem;

using namespace tl;

TEST(AprilTagBoard, Kaess)
{
    const fs::path root{"apriltag"};

    /// Sample 1: 36h11, 6x14, 3 boards, starts from 0, 90, 180
    const auto img1Path = root / "36h11_6x14_0-90-180.png";
    if (!fs::exists(img1Path)) {
        FAIL() << "Sample image 1 doesnt exist: " << img1Path;
    }

    const auto img1 = cv::imread(img1Path.string());
    if (img1.empty()) {
        FAIL() << "Sample image 1 is invalid: " << img1Path;
    }

    KalibrAprilTagBoard::Options opts;
    opts.tagRows = 6;
    opts.tagCols = 14;
    opts.tagSize = 0.046;
    opts.tagSpacingRatio = 0.3;
    opts.startId = 0;
    KalibrAprilTagBoard::DetectOptions detectOpts;

    const KalibrAprilTagBoard board{opts, detectOpts};

    cv::Mat viz1;
    const auto detection = board.computeObservation(img1, viz1);

    cv::imwrite("apriltag_detection1.png", viz1);

    EXPECT_TRUE(detection.valid()) << "Failed to detect target.";
}

TEST(AprilTagBoard, AprilTag3)
{
    GTEST_SKIP() << "Not implemented.";
    // ...
}
