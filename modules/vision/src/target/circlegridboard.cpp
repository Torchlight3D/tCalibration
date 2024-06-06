#include "circlegridboard.h"

#include <numeric>

#include <opencv2/calib3d.hpp>

namespace tl {

//------- CircleGridBoard starts from here
CircleGridBoard::CircleGridBoard(const Options &opts)
    : CalibBoardBase(opts.rows, opts.cols),
      m_options(opts),
      m_spacing(opts.spacing)
{
    // FIXME:
    // 1. Check spacing is positive
    // 2. Resolve call virtual method in ctor warning
    createBoardPoints();
}

double CircleGridBoard::tagDistance() const { return m_spacing; }

double CircleGridBoard::tagSize() const { return m_spacing; }

TargetDetection CircleGridBoard::computeObservation(cv::InputArray _img,
                                                    cv::OutputArray _viz) const
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
        cv::findCirclesGrid(_img, patternSize(), centers_f, flags);

    if (!found) {
        return {};
    }

    if (_viz.needed()) {
        _img.copyTo(_viz);
        cv::drawChessboardCorners(_viz.getMat(), patternSize(), centers_f,
                                  true);
    }

    std::vector<cv::Point2d> centers;
    std::transform(centers_f.begin(), centers_f.end(),
                   std::back_inserter(centers),
                   [](const auto &pt) { return cv::Point2d(pt); });

    return {.corners = centers,
            .cornerIds = cornerIds(),
            .detected = std::vector<uchar>(cornerCount(), 1),
            .count = cornerCount()};
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

namespace key {
constexpr char kRows[]{"rows"};
constexpr char kCols[]{"cols"};
constexpr char kSpacing[]{"spacing"};
// CircleGridBoard
constexpr char kAsymetricGrid[]{"asymetric_grid"};
} // namespace key

void to_json(nlohmann::json &j, const CircleGridBoard::Options &opts)
{
    j[key::kRows] = opts.rows;
    j[key::kCols] = opts.cols;
    j[key::kSpacing] = opts.spacing;
    j[key::kAsymetricGrid] = opts.asymmetricGrid;
}

void from_json(const nlohmann::json &j, CircleGridBoard::Options &opts)
{
    j.at(key::kRows).get_to(opts.rows);
    j.at(key::kCols).get_to(opts.cols);
    j.at(key::kSpacing).get_to(opts.spacing);
    j.at(key::kAsymetricGrid).get_to(opts.asymmetricGrid);
}

} // namespace tl
