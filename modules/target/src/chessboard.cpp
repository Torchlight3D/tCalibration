#include "chessboard.h"

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

Chessboard::Chessboard(const Options& opts, const DetectOptions& detectOpts)
    : CalibBoardBase(opts.rows, opts.cols),
      m_detectOpts(detectOpts),
      m_rowSpacing(opts.rowSpacing),
      m_colSpacing(opts.colSpacing)
{
    // TODO:
    // 1. Check spacing is positive
    // 2. Resolve call virtual method in ctor warning
    createBoardPoints();
}

double Chessboard::tagDistance() const { return m_rowSpacing; }

double Chessboard::tagSize() const { return m_rowSpacing; }

TargetDetection Chessboard::computeObservation(cv::InputArray image,
                                               cv::OutputArray _viz) const
{
    // NOTE: cv::findChessboardCornersXX will take care of type conversion.
#if CV_VERSION_MAJOR == 3
    // 1. Find corners
    const int flags = [this] {
        int flags{0};
        if (m_detectOpts.useFastCheck)
            flags |= cv::CALIB_CB_FAST_CHECK;
        if (m_detectOpts.useAdaptiveThreshold)
            flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
        if (m_detectOpts.normalizeImage)
            flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
        if (m_detectOpts.filterQuads)
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
    if (m_detectOpts.doSubPix && found) {
        // NOTE: cv::cornerSubPix wont take care of type conversion.
        cv::Mat gray;
        if (image.channels() != 1) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        }
        else {
            gray = image.getMat();
        }

        const cv::Size winSize{m_detectOpts.subPixWinSize,
                               m_detectOpts.subPixWinSize};
        const cv::Size zeroZone{-1, -1};
        const cv::TermCriteria criteria{
            cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1};

        cv::cornerSubPix(gray, corners_f, winSize, zeroZone, criteria);
    }
#elif CV_VERSION_MAJOR == 4
    constexpr auto flags = cv::CALIB_CB_ACCURACY;
    // NOTE: findChessboardCornersSB use Point2f, but not mentioned in the doc.
    std::vector<cv::Point2f> corners_f;
    const bool found =
        cv::findChessboardCornersSB(image, patternSize(), corners_f, flags);

    if (!found) {
        return {};
    }
#endif

    if (_viz.needed()) {
        image.copyTo(_viz);
        cv::drawChessboardCorners(_viz.getMat(), patternSize(), corners_f,
                                  true);
    }

    std::vector<cv::Point2d> corners;
    corners.reserve(corners_f.size());
    std::transform(corners_f.begin(), corners_f.end(),
                   std::back_inserter(corners),
                   [](const auto& pt) { return (cv::Point2d)pt; });

    // Detect all or nothing
    return {.corners = corners,
            .cornerIds = cornerIds(),
            .detected = std::vector<uchar>(cornerCount(), 1),
            .count = cornerCount()};
}

void Chessboard::createBoardPoints()
{
    m_boardPoints.clear();
    for (int r{0}; r < m_rows; r++) {
        for (int c{0}; c < m_cols; c++) {
            m_boardPoints.emplace_back(m_rowSpacing * r, m_colSpacing * c, 0.);
        }
    }

    std::iota(m_ids.begin(), m_ids.end(), 0);
}

namespace key {
constexpr char kRows[]{"rows"};
constexpr char kCols[]{"cols"};
constexpr char kRowSpacing[]{"row_spacing"};
constexpr char kColSpacing[]{"col_spacing"};
// Chessboard
constexpr char kSubPixWindowSize[]{"subpix_win_size"};
constexpr char kUseAdaptiveThreshold[]{"use_adaptive_threshold"};
constexpr char kUseFastCheck[]{"use_fast_check"};
constexpr char kFilterQuads[]{"filter_quads"};
constexpr char kNormalizeImage[]{"normalize_image"};
} // namespace key

void to_json(nlohmann::json& j, const Chessboard::Options& opts)
{
    j[key::kRows] = opts.rows;
    j[key::kCols] = opts.cols;
    j[key::kRowSpacing] = opts.rowSpacing;
    j[key::kColSpacing] = opts.colSpacing;
}

void from_json(const nlohmann::json& j, Chessboard::Options& opts)
{
    j.at(key::kRows).get_to(opts.rows);
    j.at(key::kCols).get_to(opts.cols);
    j.at(key::kRowSpacing).get_to(opts.rowSpacing);
    j.at(key::kColSpacing).get_to(opts.colSpacing);
}

void to_json(nlohmann::json& j, const Chessboard::DetectOptions& opts)
{
    if (opts.doSubPix) {
        j[key::kSubPixWindowSize] = opts.subPixWinSize;
    }
    else {
        j[key::kSubPixWindowSize] = nullptr;
    }
    j[key::kUseAdaptiveThreshold] = opts.useAdaptiveThreshold;
    j[key::kUseFastCheck] = opts.useFastCheck;
    j[key::kFilterQuads] = opts.filterQuads;
    j[key::kNormalizeImage] = opts.normalizeImage;
}

void from_json(const nlohmann::json& j, Chessboard::DetectOptions& opts)
{
    if (const auto val = j.at(key::kSubPixWindowSize); val.is_null()) {
        opts.doSubPix = false;
        opts.subPixWinSize = -1;
    }
    else {
        opts.doSubPix = true;
        j.at(key::kSubPixWindowSize).get_to(opts.subPixWinSize);
    }
    j.at(key::kUseAdaptiveThreshold).get_to(opts.useAdaptiveThreshold);
    j.at(key::kUseFastCheck).get_to(opts.useFastCheck);
    j.at(key::kFilterQuads).get_to(opts.filterQuads);
    j.at(key::kNormalizeImage).get_to(opts.normalizeImage);
}

} // namespace tl
