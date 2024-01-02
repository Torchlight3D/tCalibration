#include <filesystem>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <AxCalibTarget/AprilTagBoard>

namespace fs = std::filesystem;
using namespace thoht;

TEST(AprilTag, Dataset)
{
    const std::string dataset_path{"/home/bobblelaw/Data/OneFive_trial1"};

    const fs::path root{dataset_path};
    const auto leftpath = root / "left";
    const auto rightpath = root / "right";

    const AprilTagBoard board{6, 6, 0.14, 0.042};

    constexpr int kCount = 20;
    int count{0};
    for (const auto& entry : fs::directory_iterator{leftpath}) {
        if (!entry.is_regular_file() || entry.path().extension() != ".png") {
            continue;
        }

        const auto image = cv::imread(entry.path().string());
        const auto detections = board.computeObservation(image);

        std::ostringstream oss;
        oss << "Ids: ";
        for (const auto& id : detections.cornerIds) {
            oss << id << ", ";
        }
        oss << ".";
        LOG(INFO) << oss.str();

        count++;
        if (count == kCount) {
            break;
        }
    }

    EXPECT_TRUE(true);
}
