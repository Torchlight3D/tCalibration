#pragma once

#include "calib_board_base.h"

namespace thoht {

class CircleGridBoard : public CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<CircleGridBoard>;
    using ConstPtr = std::shared_ptr<const CircleGridBoard>;

    struct Options
    {
        bool asymmetricGrid{false};

        Options() {}
    };

    // FIXME: A circular board should be defined by
    // 1. Symetric, center spacing and circle diameter
    // 2. Asymetric, diagonal center spacing and circle diameter
    CircleGridBoard(int rows, int cols, double spacing,
                    const Options& options = {});
    virtual ~CircleGridBoard() = default;

    double tagDistance() const override;
    double tagSize() const override;

    TargetDetection computeObservation(cv::InputArray image) const override;

    void drawDetection(const TargetDetection& detection,
                       cv::Mat& imgResult) const override;

protected:
    void createBoardPoints() override;

private:
    CircleGridBoard::Options m_options;
    double m_spacing; // meter
};

} // namespace thoht
