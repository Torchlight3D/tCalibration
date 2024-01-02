#pragma once

#include "calib_board_base.h"

namespace apriltags {
class TagDetector;
}

namespace thoht {

class AprilTagBoard : public CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<AprilTagBoard>;
    using ConstPtr = std::shared_ptr<const AprilTagBoard>;

    // TODO:
    // 1. Add TagFamily enum
    // 2. Add TagCode as parameter
    struct Options
    {
        double maxSubpixDisplacement{1.5};
        double minBorderDistance{4.};
        int minTagsForValidObs{4};
        int blackTagBorder{2};
        bool doSubpixRefinement{true};

        Options() {}
    };

    // NOTE: tagSpacing is a ratio!!!
    AprilTagBoard(int tagRows, int tagCols, double tagSize, double tagSpacing,
                  const Options& options = {});
    ~AprilTagBoard();

    /// Properties
    double tagDistance() const override;
    double tagSize() const override;

    /// Actions
    TargetDetection computeObservation(cv::InputArray image) const override;

    void drawDetection(const TargetDetection& detection,
                       cv::Mat& imgResult) const override;

protected:
    void createBoardPoints() override;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

} // namespace thoht
