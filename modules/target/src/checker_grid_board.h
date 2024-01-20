#pragma once

#include "calib_board_base.h"

namespace tl {

class CheckerBoard : public CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<CheckerBoard>;
    using ConstPtr = std::shared_ptr<const CheckerBoard>;

    struct Options
    {
        // Subpixel post-process, not neccessary after OpenCV 4
        int subPixWinSize{11};
        bool doSubPix{true};

        // OpenCV parameters
        bool useAdaptiveThreshold{true};
        bool useFastCheck{true};
        bool filterQuads{false};
        bool normalizeImage{true};

        Options() {}
    };

    // FIXME: A checker board should be defined only by checker width
    CheckerBoard(int rows, int cols, double rowSpacing, double colSpacing,
                 const Options& options = {});
    virtual ~CheckerBoard() = default;

    double tagDistance() const override;
    double tagSize() const override;

    TargetDetection computeObservation(cv::InputArray image) const override;

    void drawDetection(const TargetDetection& detection,
                       cv::Mat& imgResult) const override;

protected:
    void createBoardPoints() override;

private:
    CheckerBoard::Options m_options;
    double m_rowSpacing; // meter
    double m_colSpacing; // meter
};

} // namespace tl
