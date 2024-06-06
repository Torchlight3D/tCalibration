#pragma once

#include "calibboardbase.h"

#include <json/json.hpp>

namespace tl {

// Brief:
// AprilTagBoard used by Kalibr
class KalibrAprilTagBoard final : public CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<KalibrAprilTagBoard>;
    using ConstPtr = std::shared_ptr<const KalibrAprilTagBoard>;

    struct Options
    {
        // Number of tags in row, which makes row corner count 2*tagRows
        int tagRows = 6;

        // Number of tags in column, which makes col corner count 2*tagCols
        int tagCols = 6;

        // Tag size in meter
        double tagSize = 0.105;

        // Tag spacing as the ratio to tag size
        double tagSpacingRatio = 0.3;

        // Black margin around the code pattern
        // NOTE: When AprilTag v3 is enable, this border will be ignored.
        int blackBorder = 2;

        int startId = 0;

        Options() {}
    };

    struct DetectOptions
    {
        // Subpixel refinement of extracted corners
        bool doSubpixRefinement = true;

        // Max displacement squarred in subpixel refinement
        double maxSubpixDisplacement = 1.224745;

        // Min number of tags for a valid observation
        int minDetectTagCount = 4;

        // Min detected rate of corners for a valid observation
        double minDetectRate = 0.05;

        // Min. distance form image border for valid points [px]
        float minBorderDistance = 4.f;

        DetectOptions() {}
    };

    explicit KalibrAprilTagBoard(const Options& opts = {},
                                 const DetectOptions& detectOpts = {});
    ~KalibrAprilTagBoard();

    inline static constexpr char kType[20]{"KalibrAprilTagBoard"};

    double tagDistance() const override;
    double tagSize() const override;

    TargetDetection computeObservation(
        cv::InputArray image,
        cv::OutputArray viz = cv::noArray()) const override;

private:
    void createBoardPoints() override;

private:
    class Impl;
    const std::unique_ptr<Impl> d;
};

// nlohmann::json interface
void to_json(nlohmann::json& json, const KalibrAprilTagBoard::Options& opts);
void from_json(const nlohmann::json& json, KalibrAprilTagBoard::Options& opts);

void to_json(nlohmann::json& json,
             const KalibrAprilTagBoard::DetectOptions& opts);
void from_json(const nlohmann::json& json,
               KalibrAprilTagBoard::DetectOptions& opts);

} // namespace tl
