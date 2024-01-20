#include "checker_grid_board.h"

#include <numeric>

#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace tl {

/// \brief A checkerboard example
///          *-------*-------*-------*
///          | BLACK | WHITE | BLACK |
///          *------(3)-----(4)------*
///          | WHITE | BLACK | WHITE |
///          *------(1)-----(2)------*
///    y     | BLACK | WHITE | BLACK |
///   ^      *-------*-------*-------*
///   |-->x
///

CheckerBoard::CheckerBoard(int rows, int cols, double rowSpacing,
                           double colSpacing, const Options &options)
    : CalibBoardBase(rows, cols),
      m_options(options),
      m_rowSpacing(rowSpacing),
      m_colSpacing(colSpacing)
{
    // TODO:
    // 1. Check spacing is positive
    // 2. Resolve call virtual method in ctor warning
    createBoardPoints();
}

double CheckerBoard::tagDistance() const { return m_rowSpacing; }

double CheckerBoard::tagSize() const { return m_rowSpacing; }

TargetDetection CheckerBoard::computeObservation(cv::InputArray image) const
{
    // NOTE: cv::findChessboardCornersXX will take care of type conversion.
#if CV_VERSION_MAJOR == 3
    // 1. Find corners
    const int flags = [this] {
        int flags{0};
        if (m_options.useFastCheck)
            flags |= cv::CALIB_CB_FAST_CHECK;
        if (m_options.useAdaptiveThreshold)
            flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
        if (m_options.normalizeImage)
            flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
        if (m_options.filterQuads)
            flags |= cv::CALIB_CB_FILTER_QUADS;

        return flags;
    }();

    std::vector<cv::Point2f> corners_f;
    const bool found =
        cv::findChessboardCorners(image, patternSize(), corners_f, flags);

    if (!found) {
        return {};
    }

    // 2. SubPix refinement
    if (m_options.doSubPix && found) {
        // NOTE: cv::cornerSubPix wont take care of type conversion.
        cv::Mat gray;
        if (image.channels() != 1) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        }
        else {
            gray = image.getMat();
        }

        const cv::Size winSize{m_options.subPixWinSize,
                               m_options.subPixWinSize};
        const cv::Size zeroZone{-1, -1};
        const cv::TermCriteria criteria{
            cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1};

        cv::cornerSubPix(gray, corners_f, winSize, zeroZone, criteria);
    }
#elif CV_VERSION_MAJOR == 4
    constexpr int flags = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY;
    // NOTE: findChessboardCornersSB use Point2f, but not mentioned in the doc.
    std::vector<cv::Point2f> corners_f;
    const bool found =
        cv::findChessboardCornersSB(image, patternSize(), corners_f, flags);

    if (!found) {
        return {};
    }
#endif

    std::vector<cv::Point2d> corners;
    corners.reserve(corners_f.size());
    std::transform(corners_f.begin(), corners_f.end(),
                   std::back_inserter(corners),
                   [](const auto &pt) { return (cv::Point2d)pt; });

    return {.cornerIds = cornerIds(), .corners = corners, .valid = true};
}

void CheckerBoard::drawDetection(const TargetDetection &detection,
                                 cv::Mat &imgResult) const
{
    // FIXME: corners should be cv::Point3f
    return cv::drawChessboardCorners(imgResult, patternSize(),
                                     detection.corners, detection.valid);
}

void CheckerBoard::createBoardPoints()
{
    m_boardPoints.clear();
    for (int r{0}; r < m_rows; r++) {
        for (int c{0}; c < m_cols; c++) {
            m_boardPoints.emplace_back(m_rowSpacing * r, m_colSpacing * c, 0.);
        }
    }

    std::iota(m_ids.begin(), m_ids.end(), 0);
}

} // namespace tl
