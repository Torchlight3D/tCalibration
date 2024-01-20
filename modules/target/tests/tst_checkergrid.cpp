// #include <gtest/gtest.h>

// #include <opencv2/core/mat.hpp>
// #include <opencv2/imgproc.hpp>

// #include <tTarget/CheckerGridBoard>

// using namespace tl;

// TEST(CheckerGridBoard, Overall)
//{
//     try {
//         int rows = 8;
//         int cols = 9;
//         double rowSpacingMeters = 0.05;
//         double colSpacingMeter = 0.05;

//        // create a target:
//        CheckerBoard::Options target_options;
//        target_options.doSubPix = true;
//        auto target = std::make_shared<CheckerBoard>(
//            rows, cols, rowSpacingMeters, colSpacingMeter, target_options);

//        // create a camera:
//        boost::shared_ptr<DistortedPinholeRsCameraGeometry> geometry(
//            new DistortedPinholeRsCameraGeometry());

//        // create a detector:
//        GridDetector::GridDetectorOptions detector_options;
//        GridDetector detector(geometry, target, detector_options);

//        // load the test image and extract the target:
//        cv::Mat image;
//        image = cv::imread("testImageCheckerboard.jpg", 0); // force grayscale
//        if (!image.data) {
//            LOG(INFO) << "Checkerboard test image not found";
//        }
//        ASSERT_TRUE(image.data);

//        // initialise the camera geometry:
//        ASSERT_TRUE(detector.initCameraGeometryFromObservation(image));

//        // extract the target
//        GridCalibrationTargetObservation obs;
//        ASSERT_TRUE(detector.findTarget(image, obs));

//        // try to get the intrinsics
//        Eigen::Matrix3d cam = geometry->projection().getCameraMatrix();
//        SM_DEBUG_STREAM("Camera matrix:\n" << cam << std::endl);

//        // test the transformation:
//        sm::kinematics::Transformation transformation;
//        ASSERT_TRUE(
//            geometry->projection().estimateTransformation(obs,
//            transformation));

//        SCOPED_TRACE("");
//    }
//    catch (const std::exception &e) {
//        FAIL() << e.what();
//    }
//}
