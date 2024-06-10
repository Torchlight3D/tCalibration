#include "stereorectify.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/imgproc.hpp>

#include <tCamera/FisheyeCameraModel>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCamera/PinholeRadialTangentialCameraModel>

namespace tl {

bool initUndistortRectifyMap(const Camera& left, const Camera& right,
                             const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                             cv::OutputArray map1_left,
                             cv::OutputArray map2_left,
                             cv::OutputArray map1_right,
                             cv::OutputArray map2_right)
{
    if (left.cameraIntrinsicsModel() != right.cameraIntrinsicsModel()) {
        LOG(ERROR)
            << "Failed to initialize undistortion map: "
               "Only stereo camera with identical camera model is supported.";
        return false;
    }

    cv::Mat rmat, tvec;
    cv::eigen2cv(R, rmat);
    cv::eigen2cv(t, tvec);

    switch (left.cameraIntrinsicsModel()) {
        case CameraIntrinsicsType::Omnidirectional: {
            cv::Mat R1, R2;
            cv::omnidir::stereoRectify(rmat, tvec, R1, R2);

            auto leftParams = static_cast<const OmnidirectionalCameraModel*>(
                left.cameraIntrinsics().get());
            // clang-format off
            cv::Matx33d K1{leftParams->fx(),  0., leftParams->cx(),
                           0.,  leftParams->fy(), leftParams->cy(),
                           0.,                0.,              1.};
            // clang-format on
            cv::Vec4d D1{leftParams->k1(), leftParams->k2(), leftParams->p1(),
                         leftParams->p2()};
            cv::Mat xi1 = (cv::Mat_<double>(1, 1) << leftParams->xi());

            auto rightParams = static_cast<const OmnidirectionalCameraModel*>(
                right.cameraIntrinsics().get());
            // clang-format off
            cv::Matx33d K2{rightParams->fx(),  0., rightParams->cx(),
                           0.,  rightParams->fy(), rightParams->cy(),
                           0.,                 0.,               1.};
            // clang-format on
            cv::Vec4d D2{rightParams->k1(), rightParams->k2(),
                         rightParams->p1(), rightParams->p2()};
            cv::Mat xi2 = (cv::Mat_<double>(1, 1) << rightParams->xi());

            constexpr double kMagicFocalLength = 314.;
            // clang-format off
            cv::Matx33d Knew{kMagicFocalLength, 0., (leftParams->cx() + rightParams->cx()) * 0.5,
                             0., kMagicFocalLength, (leftParams->cy() + rightParams->cy()) * 0.5,
                             0.,                0.,                                           1.};
            // clang-format on

            constexpr int m1type = CV_32FC1;
            constexpr int flags = cv::omnidir::RECTIFY_PERSPECTIVE;
            const cv::Size size{left.imageWidth(), left.imageHeight()};

            cv::omnidir::initUndistortRectifyMap(K1, D1, xi1, R1, Knew, size,
                                                 m1type, map1_left, map2_left,
                                                 flags);
            cv::omnidir::initUndistortRectifyMap(K2, D2, xi2, R2, Knew, size,
                                                 m1type, map1_right, map2_right,
                                                 flags);
            return true;
        } break;
        case CameraIntrinsicsType::Fisheye: {
            auto leftParams = static_cast<const FisheyeCameraModel*>(
                left.cameraIntrinsics().get());
            // clang-format off
            cv::Matx33d K1{leftParams->fx(), 0., leftParams->cx(),
                           0,  leftParams->fy(), leftParams->cy(),
                           0.,               0.,               1.};
            // clang-format on
            cv::Vec4d D1{leftParams->k1(), leftParams->k2(), leftParams->k3(),
                         leftParams->k4()};

            auto rightParams = static_cast<const FisheyeCameraModel*>(
                right.cameraIntrinsics().get());
            // clang-format off
            cv::Matx33d K2{rightParams->fx(), 0., rightParams->cx(),
                           0,  rightParams->fy(), rightParams->cy(),
                           0.,                0.,               1.};
            // clang-format on
            cv::Vec4d D2{rightParams->k1(), rightParams->k2(),
                         rightParams->k3(), rightParams->k4()};

            constexpr int flags = cv::CALIB_ZERO_DISPARITY;
            const cv::Size size{left.imageWidth(), left.imageHeight()};

            cv::Mat R1, R2, P1, P2, Q;
            cv::fisheye::stereoRectify(K1, D1, K2, D2, size, rmat, tvec, R1, R2,
                                       P1, P2, Q, flags);

            constexpr int m1type = CV_32FC1;

            cv::Mat mapX1, mapY1;
            cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, size, m1type,
                                                 map1_left, map2_left);
            cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, size, m1type,
                                                 map1_right, map2_right);

            return true;
        } break;
        case CameraIntrinsicsType::PinholeRadialTangential: {
            auto leftParams =
                static_cast<const PinholeRadialTangentialCameraModel*>(
                    left.cameraIntrinsics().get());
            // clang-format off
            cv::Matx33d K1{leftParams->fx(), 0., leftParams->cx(),
                           0,  leftParams->fy(), leftParams->cy(),
                           0.,               0.,              1.};
            // clang-format on
            cv::Vec4d D1{leftParams->k1(), leftParams->k2(), leftParams->t1(),
                         leftParams->t2()};

            auto rightParams =
                static_cast<const PinholeRadialTangentialCameraModel*>(
                    right.cameraIntrinsics().get());
            // clang-format off
            cv::Matx33d K2{rightParams->fx(), 0., rightParams->cx(),
                           0,  rightParams->fy(), rightParams->cy(),
                           0.,                0.,               1.};
            // clang-format on
            cv::Vec4d D2{rightParams->k1(), rightParams->k2(),
                         rightParams->t1(), rightParams->t2()};

            constexpr int flags = cv::CALIB_ZERO_DISPARITY;
            constexpr double alpha = -1.;
            const cv::Size size{left.imageWidth(), left.imageHeight()};

            cv::Mat R1, R2, P1, P2, Q;
            cv::stereoRectify(K1, D1, K2, D2, size, rmat, tvec, R1, R2, P1, P2,
                              Q, flags, alpha);

            constexpr int m1type = CV_32FC1;

            cv::initUndistortRectifyMap(K1, D1, R1, P1, size, m1type, map1_left,
                                        map2_left);
            cv::initUndistortRectifyMap(K2, D2, R2, P2, size, m1type,
                                        map1_right, map2_right);

            return true;
        } break;
        default: {
        } break;
    }

    return false;
}

void io::toStereoUndistortRectifyMapString(cv::InputArray map1_left,
                                           cv::InputArray map2_left,
                                           cv::InputArray map1_right,
                                           cv::InputArray map2_right,
                                           std::string& left_str,
                                           std::string& right_str)
{
    // NOTE: Take from stereo_calib_for_x3.cpp
    auto mapXYToString = [](cv::InputArray map1,
                            cv::InputArray map2) -> std::string {
        // TODO: Check type and size?
        const cv::Mat mapx = map1.getMat();
        const cv::Mat mapy = map2.getMat();
        const int rows = mapx.rows;
        const int cols = mapx.cols;

        std::ostringstream oss;

        // Meta
        oss << std::to_string(1) << "\n";
        oss << std::to_string(50) << " " << std::to_string(50) << "\n";
        oss << std::to_string(rows) + " " + std::to_string(cols) << "\n";
        oss << std::to_string((rows - 1) / 2.) + " " +
                   std::to_string((cols - 1) / 2.)
            << "\n";

        // Mapping
        for (int r{0}; r < rows; r++) {
            for (int c{0}; c < cols; c++) {
                oss << std::to_string(
                           static_cast<int>(round(mapy.at<float>(r, c)))) +
                           ":" +
                           std::to_string(
                               static_cast<int>(round(mapx.at<float>(r, c)))) +
                           " ";
            }
            oss << "\n";
        }

        return oss.str();
    };

    left_str = mapXYToString(map1_left, map2_left);
    right_str = mapXYToString(map1_right, map2_right);
}

} // namespace tl
