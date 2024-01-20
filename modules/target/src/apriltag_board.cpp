#include "apriltag_board.h"

#include <cassert>
#include <numeric>

#include <glog/logging.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "apriltag/tag36h11.h"
#include "apriltag/tag_detector.h"

namespace tl {

using namespace apriltags;

class AprilTagBoard::Impl
{
public:
    explicit Impl(AprilTagBoard *q);

    void init(int tagRows, int tagCols, double tagSize, double tagSpacing,
              const Options &options);

    std::array<CornerId, 4> cornerIdsFromTagId(int tagId) const;
    int tagIdFromCornerId(CornerId id) const;

public:
    AprilTagBoard *const q;
    AprilTagBoard::Options m_options;
    std::unique_ptr<apriltags::TagDetector> m_tagDetector;
    std::vector<TagDetection> m_detections;
    double m_tagSize;
    double m_tagSpacing;
};

AprilTagBoard::Impl::Impl(AprilTagBoard *q) : q(q) {}

void AprilTagBoard::Impl::init(int tagRows, int tagCols, double tagSize,
                               double tagSpacing, const Options &options)
{
    m_options = options;
    m_tagDetector =
        std::make_unique<TagDetector>(kTagCodes36h11, m_options.blackTagBorder);
    m_tagSize = tagSize;
    m_tagSpacing = tagSpacing;

    // TODO: Reserve detection cache or not?
}

std::array<CornerId, 4> AprilTagBoard::Impl::cornerIdsFromTagId(int tagId) const
{
    int tagCols = q->cols() / 2;
    int col = tagId % tagCols;
    int row = tagId / tagCols;
    // TODO: Assume start from 0, what if not.
    int bl = row * 2 * q->cols() + col * 2;

    return {bl, bl + 1, bl + q->cols(), bl + q->cols() + 1};
}

int AprilTagBoard::Impl::tagIdFromCornerId(CornerId id) const
{
    int col = id % q->cols() / 2;
    int row = id / q->cols() / 2;
    return row * (q->cols() / 2) + col;
}

/// \brief An april grid board example
///          12-----13  14-----15
///          | TAG 3 |  | TAG 4 |
///          8-------9  10-----11
///          4-------5  6-------7
///    y     | TAG 1 |  | TAG 2 |
///   ^      0-------1  2-------3  <- CornerId
///   |-->x

AprilTagBoard::AprilTagBoard(int tagRows, int tagCols, double tagSize,
                             double tagSpacing, const Options &options)
    : CalibBoardBase(2 * tagRows, 2 * tagCols), // 4 corners per tag
      d(std::make_unique<Impl>(this))
{
    assert(tagSpacing > 0. && tagSize > 0.);

    d->init(tagRows, tagCols, tagSize, tagSpacing, options);
    createBoardPoints();
}

AprilTagBoard::~AprilTagBoard() = default;

double AprilTagBoard::tagDistance() const
{
    return d->m_tagSpacing * d->m_tagSize;
}

double AprilTagBoard::tagSize() const { return d->m_tagSize; }

TargetDetection AprilTagBoard::computeObservation(cv::InputArray image) const
{
    // 0. Preprocess
    const auto type = image.type();
    const auto depth = CV_MAT_DEPTH(type);
    const auto channel = CV_MAT_CN(type);
    CV_CheckType(type, depth == CV_8U && (channel == 1 || channel == 3),
                 "Only 8-bit grayscale or color images are supported");

    cv::Mat img;
    if (image.channels() != 1) {
        cv::cvtColor(image, img, cv::COLOR_BGR2GRAY);
    }
    else {
        img = image.getMat();
    }

    // 1. Detect the tags. NOTE: d here should be a const pointer
    d->m_detections = d->m_tagDetector->extractTags(img);

    // 2. Remove bad detections
    const cv::Rect2d validArea{
        cv::Point2d{d->m_options.minBorderDistance,
                    d->m_options.minBorderDistance},
        cv::Point2d{img.cols - d->m_options.minBorderDistance,
                    img.rows - d->m_options.minBorderDistance}};

    for (auto it = d->m_detections.begin(); it != d->m_detections.end();) {
        bool remove{false};

        // a. Check all four corners completely inside image
        remove |= std::any_of(std::begin(it->p), std::end(it->p),
                              [&validArea](const auto &pt) {
                                  return !validArea.contains({pt.x, pt.y});
                              });

        // b. Remove tags flagged as bad
        remove |= !it->good;

        // c. Remove tags with invalid ID.
        // FIXME: Failed if ids dont start from 0
        remove |= (it->id >= cornerCount() / 4);

        it = remove ? d->m_detections.erase(it) : it + 1;
    }

    // 3. Check enough observations
    if (d->m_detections.size() < d->m_options.minTagsForValidObs) {
        LOG(ERROR) << "Not enough detected tags after removing bad detections.";
        return {};
    }

    // 4. Remove tags with duplicated id
    std::sort(d->m_detections.begin(), d->m_detections.end(),
              TagDetection::sortById);

    if (d->m_detections.size() > 1) {
        for (size_t i{0}; i < d->m_detections.size() - 1; ++i) {
            if (d->m_detections[i].id == d->m_detections[i + 1].id) {
                LOG(ERROR) << "Wild AprilTag detected. Abort...";
                return {};
            }
        }
    }

    // 5. Subpixel refinement
    // a. computer subpix refine search radius depending on size of tag
    // NOTE: Currently not used
    std::vector<double> searchRadii;
    searchRadii.reserve(d->m_detections.size());

    constexpr bool adaptiveSearchRadius{false};
    if constexpr (adaptiveSearchRadius) {
        std::transform(
            d->m_detections.begin(), d->m_detections.end(),
            std::back_inserter(searchRadii),
            [](const auto &detection) -> double {
                constexpr double kMinRadius{2.};
                constexpr double kRatioToEdge{0.075};
                const double avgEdge = detection.observedPerimeter / 4.;
                return std::max(kMinRadius, avgEdge * kRatioToEdge - 1.);
            });
    }

    // b. data conversion
    // TODO: Delete after refactor AprilTags code
    std::vector<cv::Point2f> cornersRaw;
    std::vector<CornerId> cornerRawIds;
    cornersRaw.reserve(4 * d->m_detections.size());
    for (const auto &detection : d->m_detections) {
        for (int j{0}; j < 4; ++j) {
            cornersRaw.emplace_back(detection.p[j].x, detection.p[j].y);
        }

        const auto cornerId = d->cornerIdsFromTagId(detection.id);
        cornerRawIds.insert(cornerRawIds.end(), cornerId.begin(),
                            cornerId.end());
    }

    auto cornersRefined = cornersRaw;
    if (d->m_options.doSubpixRefinement) {
        // TODO: use different search radius
        cv::cornerSubPix(
            img, cornersRefined, {2, 2}, {-1, -1},
            cv::TermCriteria{cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                             30, 0.1});
    }

    // 7. Output
    std::vector<cv::Point2d> corners;
    std::vector<CornerId> cornerIds;
    for (size_t i{0}; i < cornersRaw.size(); ++i) {
        const auto offset = cv::norm(cornersRaw[i] - cornersRefined[i]);
        const bool toofar = offset > d->m_options.maxSubpixDisplacement;

        if (!toofar) {
            corners.push_back(cv::Point2d(cornersRefined[i]));
            cornerIds.push_back(cornerRawIds[i]);
        }
    }

    return {.cornerIds = cornerIds, .corners = corners, .valid = true};
}

void AprilTagBoard::drawDetection(const TargetDetection &tagDetection,
                                  cv::Mat &imgResult) const
{
    for (const auto &detection : d->m_detections) {
        detection.draw(imgResult);
    }
}

void AprilTagBoard::createBoardPoints()
{
    m_boardPoints.clear();
    for (int r{0}; r < m_rows; r++) {
        for (int c{0}; c < m_cols; c++) {
            m_boardPoints.emplace_back(
                (c / 2) * (1 + d->m_tagSpacing) * d->m_tagSize +
                    (c % 2) * d->m_tagSize,
                (r / 2) * (1 + d->m_tagSpacing) * d->m_tagSize +
                    (r % 2) * d->m_tagSize,
                0.);
        }
    }

    std::iota(m_ids.begin(), m_ids.end(), 0);
}

} // namespace tl
