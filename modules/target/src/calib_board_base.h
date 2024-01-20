#pragma once

#include <memory>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "target_types.h"
#include "target_detection.h"

namespace tl {

class CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<CalibBoardBase>;
    using ConstPtr = std::shared_ptr<const CalibBoardBase>;

    CalibBoardBase(int rows, int cols);
    virtual ~CalibBoardBase() = default;

    /// Properties
    // TODO: 1. consider add setter
    int rows() const;
    int cols() const;
    inline int cornerCount() const { return rows() * cols(); }
    inline cv::Size patternSize() const { return {cols(), rows()}; }

    cv::Point3d boardPoint(int index) const;
    inline auto boardPoint(int row, int col) const
    {
        return boardPoint(indexOf(row, col));
    }
    const std::vector<cv::Point3d>& boardPoints() const;
    const std::vector<CornerId>& cornerIds() const;

    cv::Point posOf(int index) const;
    int indexOf(int row, int col) const;

    virtual double tagDistance() const = 0;
    virtual double tagSize() const = 0;

    /// Actions
    virtual TargetDetection computeObservation(cv::InputArray image) const = 0;

    virtual void drawDetection(const TargetDetection& detection,
                               cv::Mat& imgResult) const = 0;

protected:
    virtual void createBoardPoints() = 0;

protected:
    std::vector<cv::Point3d> m_boardPoints;
    std::vector<CornerId> m_ids;
    int m_rows, m_cols;
};

} // namespace tl
