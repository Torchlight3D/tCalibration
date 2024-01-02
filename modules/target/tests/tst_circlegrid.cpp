// #include <gtest/gtest.h>
// #include <opencv2/core/mat.hpp>

// #include <AxCalibTarget/CircleGridBoard>

// using namespace thoht;

// TEST(CircleGridBoard, Overall)
//{
//     try {
//         int rows = 5;
//         int cols = 7;
//         double spacingMeters = 0.01;

//        // Create a target
//        CircleGridBoard::Options target_options;
//        CircleGridBoard::Ptr target(new GridCalibrationTargetCirclegrid(
//            rows, cols, spacingMeters, target_options));

//        // create a camera:
//        boost::shared_ptr<DistortedPinholeRsCameraGeometry> geometry(
//            new DistortedPinholeRsCameraGeometry());

//        // create a detector:
//        GridDetector::GridDetectorOptions detector_options;
//        GridDetector detector(geometry, target, detector_options);

//        // load the test image and extract the target:
//        cv::Mat image;
//        image = cv::imread("testImageCircleGrid.jpg", 0); // force grayscale
//        if (!image.data) {
//            std::cout << "Circlegrid test image not found" << std::endl;
//        }
//        ASSERT_TRUE(image.data);

//        // extract the target
//        GridCalibrationTargetObservation obs;
//        ASSERT_TRUE(detector.findTargetNoTransformation(image, obs));

//        SCOPED_TRACE("");
//    }
//    catch (const std::exception &e) {
//        FAIL() << e.what();
//    }
//}
