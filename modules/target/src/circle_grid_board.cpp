#include "circle_grid_board.h"

#include <numeric>
#include <opencv2/calib3d.hpp>

namespace thoht {

//------- CircleGridBoard starts from here
CircleGridBoard::CircleGridBoard(int rows, int cols, double spacing,
                                 const Options &options)
    : CalibBoardBase(rows, cols), m_options(options), m_spacing(spacing)
{
    // FIXME:
    // 1. Check spacing is positive
    // 2. Resolve call virtual method in ctor warning
    createBoardPoints();
}

double CircleGridBoard::tagDistance() const { return m_spacing; }

double CircleGridBoard::tagSize() const { return m_spacing; }

TargetDetection CircleGridBoard::computeObservation(cv::InputArray image) const
{
    const int flags = [this]() {
        int flags{0};
        if (m_options.asymmetricGrid) {
            flags |= cv::CALIB_CB_ASYMMETRIC_GRID;
        }
        else {
            flags |= cv::CALIB_CB_SYMMETRIC_GRID;
        }

        return flags;
    }();

    std::vector<cv::Point2f> centers_f;
    const bool found =
        cv::findCirclesGrid(image, patternSize(), centers_f, flags);

    std::vector<cv::Point2d> centers;
    std::transform(centers_f.begin(), centers_f.end(),
                   std::back_inserter(centers),
                   [](const auto &pt) { return cv::Point2d(pt); });

    if (found) {
        return {.cornerIds = cornerIds(), .corners = centers, .valid = found};
    }

    return {};
}

void CircleGridBoard::drawDetection(const TargetDetection &detection,
                                    cv::Mat &imgResult) const
{
    // FIXME: corners should be cv::Point3f
    cv::drawChessboardCorners(imgResult, patternSize(), detection.corners,
                              detection.valid);
}

void CircleGridBoard::createBoardPoints()
{
    m_boardPoints.clear();

    if (m_options.asymmetricGrid) {
        for (int r{0}; r < m_rows; r++) {
            for (int c{0}; c < m_cols; c++) {
                m_boardPoints.emplace_back((2 * c + r % 2) * m_spacing,
                                           r * m_spacing, 0.);
            }
        }
    }
    else {
        for (int r{0}; r < m_rows; r++) {
            for (int c{0}; c < m_cols; c++) {
                m_boardPoints.emplace_back(m_spacing * r, m_spacing * c, 0.);
            }
        }
    }

    std::iota(m_ids.begin(), m_ids.end(), 0);
}

} // namespace thoht
