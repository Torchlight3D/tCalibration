#include <filesystem>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tCore/TimeUtils>
#include <tCore/Timer>
#include <tCalibration/CameraIntrinsicsCalibration>
#include <tCalibration/StereoCameraCalibration>
#include <tTarget/KalibrAprilTagBoard>
#include <tTarget/Chessboard>

namespace fs = std::filesystem;
using namespace tl;

TEST(CameraIntrinsics, Dataset1)
{
    GTEST_SKIP() << "Skip dataset test case.";

    // 1. Create board
    // const KalibrAprilTagBoard board_{6, 6, 0.14, 0.042};

    Chessboard::Options opts;
    opts.rows = 9;
    opts.cols = 14;
    opts.rowSpacing = 0.16;
    opts.colSpacing = 0.16;
    auto board_ = std::make_shared<Chessboard>(opts);
    // const Chessboard board_{8, 10, 0.09, 0.09};

    // 3. Create player
    // FIXME: Avoid hardcode
    // const std::string dataset_path{"/home/bobblelaw/Data/One_trial1"};
    const std::string dataset_path{"/home/bobblelaw/Dataset/calib_2305171135"};

    const fs::path root{dataset_path};
    const auto leftpath = root / "left";
    const auto rightpath = root / "right";

    constexpr int kCount = 20;

    StampedTargetDetections detections1_;
    // Left
    {
        int count{0};
        for (const auto &entry : fs::directory_iterator{leftpath}) {
            if (!entry.is_regular_file() ||
                entry.path().extension() != ".png" || (count % kCount != 0)) {
                count++;
                continue;
            }

            const auto filename = entry.path().stem();
            // const auto t_s = time::nsToS(std::stoul(filename.string()));
            const auto t_s = std::stod(filename.string());

            const auto image = cv::imread(entry.path().string());
            const auto detection = board_->computeObservation(image);
            LOG(INFO) << detection.count;
            detections1_.push_back({detection, t_s});

            count++;
        }
    }

    LOG(INFO) << "Loading images from right camera.";

    StampedTargetDetections detections2_;
    // Right
    {
        int count{0};
        for (const auto &entry : fs::directory_iterator{rightpath}) {
            if (!entry.is_regular_file() ||
                entry.path().extension() != ".png" || (count % kCount != 0)) {
                count++;
                continue;
            }

            const auto filename = entry.path().stem();
            // const auto t_s = time::nsToS(std::stoul(filename.string()));
            const auto t_s = std::stod(filename.string());

            const auto image = cv::imread(entry.path().string());
            const auto detection = board_->computeObservation(image);
            LOG(INFO) << detection.count;
            detections2_.push_back({detection, t_s});

            count++;
        }
    }

    CameraIntrinsicsCalibration calib;

    auto options = calib.options();
    options.sampleDistance = board_->tagSize();
    options.imageWidth = 640;
    options.imageHeight = 480;
    options.intrinsicsType = CameraIntrinsics::Type::Pinhole;
    calib.setOptions(options);

    calib.setupScene(board_);

    calib.calibrate(detections1_, CameraId{0});
    calib.calibrate(detections2_, CameraId{1});
    return;

    // scene_->pairCamera(CameraId{0}, CameraId{1});
    // LOG(INFO) << "Stereo pair count: " << scene_->stereoViewCount();

    // Mono Result
    auto camera = calib.camera();
    LOG(INFO) << "\n"
                 "Camera: "
                 "\n"
              << *camera;

    // TODO: Camera Pose refine

    // StereoCalibration
    StereoCameraCalibration stereoCalib;
    // stereoCalib.setScene(scene_);
    stereoCalib.calibrate();

    EXPECT_TRUE(true);
}

TEST(CameraIntrinsics, OpenCV)
{
    GTEST_SKIP() << "Skip OpenCV test case.";

    const std::string dataset_path{"/home/bobblelaw/Dataset/calib_2305171135"};

    const fs::path root{dataset_path};
    const auto leftpath = root / "left";
    const auto rightpath = root / "right";

    constexpr int kCount = 20;

    constexpr double checkerSize = 0.016; // meter
    const cv::Size patternSize{14, 9};
    const cv::Size imgSize{640, 480};

    std::vector<cv::Point3d> objectPoints;
    objectPoints.reserve(patternSize.area());
    for (int y{0}; y < patternSize.height; ++y) {
        for (int x{0}; x < patternSize.width; ++x) {
            objectPoints.emplace_back(x * checkerSize, y * checkerSize, 0.);
        }
    }

    std::vector<std::vector<cv::Point3d>> objectPointsList;
    std::vector<std::vector<cv::Point2d>> left_imagePointsList;
    std::vector<std::vector<cv::Point2d>> right_imagePointsList;

    int count{0};
    for (const auto &entry : fs::directory_iterator{leftpath}) {
        if (!entry.is_regular_file() || entry.path().extension() != ".png" ||
            (count % kCount != 0)) {
            ++count;
            continue;
        }

        const auto left_filename = entry.path().filename();
        const auto right_filename = rightpath / left_filename;
        if (!fs::exists(right_filename)) {
            continue;
        }

        const auto left_img =
            cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);
        const auto right_img =
            cv::imread(right_filename.string(), cv::IMREAD_GRAYSCALE);

        const int flags = cv::CALIB_CB_LARGER | cv::CALIB_CB_MARKER |
                          cv::CALIB_CB_EXHAUSTIVE |
                          cv::CALIB_CB_NORMALIZE_IMAGE;

        std::vector<cv::Point2f> left_corners, right_corners;
        const bool left_found = cv::findChessboardCornersSB(
            left_img, patternSize, left_corners, flags);
        const bool right_found = cv::findChessboardCornersSB(
            right_img, patternSize, right_corners, flags);

        LOG(INFO) << "Found corner size in left: " << left_corners.size()
                  << ". Found corner size in right: " << right_corners.size();

        if (left_found && right_found) {
            auto toCvPoint2d = [](const std::vector<cv::Point2f> &points_f) {
                std::vector<cv::Point2d> points_d;
                points_d.reserve(points_f.size());
                std::transform(points_f.cbegin(), points_f.cend(),
                               std::back_inserter(points_d),
                               [](const auto &pt) { return cv::Point2d(pt); });
                return points_d;
            };

            left_imagePointsList.push_back(toCvPoint2d(left_corners));
            right_imagePointsList.push_back(toCvPoint2d(right_corners));
            objectPointsList.push_back(objectPoints);
        }

        ++count;
    }

    const int flags =
        cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;

    cv::Matx33d k1, k2, R;
    cv::Vec3d t;
    cv::Vec4d dist1, dist2;
    const auto rpe = cv::fisheye::stereoCalibrate(
        objectPointsList, left_imagePointsList, right_imagePointsList, k1,
        dist2, k2, dist2, imgSize, R, t, flags,
        cv::TermCriteria{cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         1e-6});

    // cv::Mat E, F;
    // const auto rpe = cv::stereoCalibrate(objectPointsList,
    // left_imagePointsList,
    //                                      right_imagePointsList, k1, dist2,
    //                                      k2, dist2, cv::Size{640, 480}, R, t,
    //                                      E, F);

    constexpr double kBalance{0.};
    constexpr double kFovScale{1.1};
    cv::Mat R1, R2, P1, P2, Q;
    cv::fisheye::stereoRectify(k1, dist1, k2, dist2, imgSize, R, t, R1, R2, P1,
                               P2, Q, cv::CALIB_ZERO_DISPARITY, imgSize,
                               kBalance, kFovScale);

    cv::Mat rmap[2][2];
    cv::fisheye::initUndistortRectifyMap(k1, dist1, R1, P1, imgSize, CV_16SC2,
                                         rmap[0][0], rmap[0][1]);
    cv::fisheye::initUndistortRectifyMap(k2, dist2, R2, P2, imgSize, CV_16SC2,
                                         rmap[1][0], rmap[1][1]);

    LOG(INFO) << "RMS reprojection error is: " << rpe << "\nRotation: " << R
              << "\nTranslation: " << t << "\nProjection matrix 1: " << k1
              << "\nDistortion 1: " << dist1 << "\nProjection matrix 2: " << k2
              << "\nDistortion 2: " << dist2;
}
