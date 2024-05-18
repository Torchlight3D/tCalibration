#pragma once

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

#include <tCore/Global>
#include <tCore/EnumUtils>
#include <tCamera/StereoCameraTypes>
#include <tCamera/coc/Camera>

namespace tl {

class StereoCameraVerification
{
public:
    // TODO:
    // 1. VerifyData should be derived or extracted from CalibData
    struct VerifyData
    {
        std::array<Eigen::Matrix3d, StereoSize> ric;
        std::array<Eigen::Vector3d, StereoSize> tic;
        std::array<camodocal::Camera::Ptr, StereoSize> cameras;

        Eigen::Matrix3d essential_c1c0;
        Eigen::Matrix3d rot_c1c0;
        Eigen::Vector3d t_c1c0;
        double focal_length;

        // TODO: Move to CalibrationIO, share common keys with save functions
        static bool loadFromFile(const std::string &filename, VerifyData &data,
                                 std::string &uuid);
        inline static bool loadFromFile(const std::string &filename,
                                        VerifyData &data)
        {
            std::string _;
            return loadFromFile(filename, data, _);
        }

        static bool loadFromText(const std::string &bytes, VerifyData &data,
                                 std::string &uuid);
        inline static bool loadFromText(const std::string &bytes,
                                        VerifyData &data)
        {
            std::string _;
            return loadFromText(bytes, data, _);
        }
    };

    struct Reference
    {
        double maxTrackerRejectRate = 10e-2;
        double maxEstimatorRejectRate = 10e-2;
        double maxStereoOptimizationCost = 20.;
        double maxRejectRelativeSizeMedian = 3e-3;
        double maxRejectRelativePositionMedian = 3e-3;
    };

    struct Options
    {
        cv::Size chessPatternSize{};
        cv::Size2d chessSize{};

        Reference reference;

        Options() {}

        // Check some of the fields are not with their default values
        bool isValid() const;
    };

    explicit StereoCameraVerification(const VerifyData &data,
                                      const Options &options = {});
    ~StereoCameraVerification();

    struct Summary
    {
        enum Item
        {
            /// VIO
            TrackerRejectRate,
            EstimatorRejectRate,
            PoseOptimizationCost,
            RejectRelativeSizeMedian,
            RejectRelativePositionMedian,

            Count,
        };

        double trackerRejectRate{-1e-2};
        double estimatorRejectRate{-1e-2};
        double stereoOptimzationCost{-1.};
        double rejectRelativeSizeMedian{-1e-3};
        double rejectRelativePostitionMedian{-1e-3};

        enum Error
        {
            NoError = 0x00,

            /// Data
            InvalidImage = 0x01,
            FailedFindChessboardCorner = 0x02,

            /// VIO
            RejectByTracker = 0x04,
            RejectByEstimator = 0x08,
            LargeOptimizationCost = 0x10, // TODO: Maybe change name
            LargeRelativeSize = 0x20,
            LargeRelativePosition = 0x40,
        };
        Error error{Error::NoError};

        bool passed() const { return error == Error::NoError; }
    };

    // This will clear existed data if there's any
    Summary startVerify(const cv::Mat &left, const cv::Mat &right);

    void drawResult(cv::InputOutputArray verifyResult,
                    cv::InputOutputArray detectionResult = cv::noArray());

private:
    DISABLE_COPY(StereoCameraVerification)

    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

MAKE_FLAGS(StereoCameraVerification::Summary::Error)

} // namespace tl
