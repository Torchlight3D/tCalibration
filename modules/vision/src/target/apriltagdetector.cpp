#include "apriltagdetector.h"

#include <numeric>
#include <set>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag36h11b2.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>

namespace tl {

namespace {

inline bool insideImage(const cv::Point2d &pt, cv::InputArray img)
{
    return cv::Rect2d{0., 0., static_cast<double>(img.cols()),
                      static_cast<double>(img.rows())}
        .contains(pt);
}

} // namespace

bool AprilTag::Board::Options::isValid() const
{
    return !tagDim.empty() && startTagId >= 0 && tagSize > 0.f &&
           tagSpacingRatio > 0.f && tagSpacingRatio < 1.f;
}

void AprilTag::Board::Detection::reserve(size_t cap)
{
    corners.reserve(cap);
    cornerIds.reserve(cap);
}

size_t AprilTag::Board::Detection::size() const { return corners.size(); }

bool AprilTag::Board::Detection::empty() const { return corners.empty(); }

bool AprilTag::Board::checkGroupValid(std::vector<Options> &options)
{
    if (options.empty()) {
        return false;
    }

    std::ranges::sort(options, [](const auto &a, const auto &b) {
        return a.startTagId < b.startTagId;
    });

    std::set<int> startIds;
    auto nextTagIdMin{0};
    for (const auto &opts : options) {
        if (!opts.isValid() || startIds.contains(opts.startTagId) ||
            opts.startTagId < nextTagIdMin) {
            return false;
        }

        startIds.emplace(opts.startTagId);
        nextTagIdMin = opts.startTagId + opts.tagDim.area();
    }

    return true;
}

AprilTag::Board::Board(const Options &opts)
{
    CHECK(opts.isValid());

    const auto numCorners = opts.tagDim.area() * 4;

    // Board points
    points.reserve(numCorners);
    const auto tagWithSpace = (1. + opts.tagSpacingRatio) * opts.tagSize;
    for (auto r{0}; r < opts.tagDim.height; r++) {
        for (auto c{0}; c < opts.tagDim.width; c++) {
            points.emplace_back((c / 2) * tagWithSpace + (c % 2) * opts.tagSize,
                                (r / 2) * tagWithSpace + (r % 2) * opts.tagSize,
                                0.f);
        }
    }

    // Board Ids
    ids.resize(numCorners, -1);
    std::iota(ids.begin(), ids.end(), opts.startTagId * 4);
}

bool AprilTag::Board::isValid() const
{
    return !points.empty() && points.size() == ids.size();
}

class AprilTag::Impl
{
public:
    Impl();
    ~Impl();

    void setupDetector(const Options &opts);
    void destroyDetector();

public:
    Options _opts;
    apriltag_family_t *_tf = NULL;   // Own
    apriltag_detector_t *_td = NULL; // Own
};

AprilTag::Impl::Impl() = default;

AprilTag::Impl::~Impl() { destroyDetector(); }

void AprilTag::Impl::setupDetector(const Options &opts)
{
    destroyDetector();

    // Initialize tag detector with options
    switch (opts.family) {
        case Family::tag16h5:
            _tf = tag16h5_create();
            break;
        case Family::tag25h9:
            _tf = tag25h9_create();
            break;
        case Family::tag36h11:
            _tf = tag36h11_create();
            break;
        case Family::tag36h11b2:
            _tf = tag36h11b2_create();
            break;
        case Family::tagCircle21h7:
            _tf = tagCircle21h7_create();
            break;
        case Family::tagCircle49h12:
            _tf = tagCircle49h12_create();
            break;
        case Family::tagStandard41h12:
            _tf = tagStandard41h12_create();
            break;
        case Family::tagStandard52h13:
            _tf = tagStandard52h13_create();
            break;
        case Family::tagCustom48h12:
            _tf = tagCustom48h12_create();
            break;
        default:
            break;
    }
    if (_tf == NULL) {
        return;
    }

    _td = apriltag_detector_create();
    apriltag_detector_add_family(_td, _tf);

    _td->quad_decimate = opts.decimate;
    _td->quad_sigma = opts.blur;
    _td->nthreads = opts.thread;
    _td->debug = false;
    _td->refine_edges = opts.refineEdges;
    _td->decode_sharpening = opts.sharpening;
}

void AprilTag::Impl::destroyDetector()
{
    if (_td != NULL) {
        apriltag_detector_destroy(_td);
    }

    if (_tf != NULL) {
        switch (_opts.family) {
            case Family::tag16h5:
                tag16h5_destroy(_tf);
                break;
            case Family::tag25h9:
                tag25h9_destroy(_tf);
                break;
            case Family::tag36h11:
                tag36h11_destroy(_tf);
                break;
            case Family::tag36h11b2:
                tag36h11b2_destroy(_tf);
                break;
            case Family::tagCircle21h7:
                tagCircle21h7_destroy(_tf);
                break;
            case Family::tagCircle49h12:
                tagCircle49h12_destroy(_tf);
                break;
            case Family::tagStandard41h12:
                tagStandard41h12_destroy(_tf);
                break;
            case Family::tagStandard52h13:
                tagStandard52h13_destroy(_tf);
                break;
            case Family::tagCustom48h12:
                tagCustom48h12_destroy(_tf);
                break;
            default:
                break;
        }
    }
}

AprilTag::AprilTag(const Options &opts) : d(std::make_unique<Impl>())
{
    setOptions(opts);
}

AprilTag::~AprilTag() = default;

void AprilTag::setOptions(const Options &opts)
{
    d->_opts = opts;
    d->setupDetector(opts);
}

std::vector<AprilTag::Detection> AprilTag::detect(cv::InputArray _img,
                                                  cv::OutputArray _viz) const
{
    if (d->_td == NULL || d->_tf == NULL) {
        return {};
    }

    const auto type = _img.type();
    const auto depth = CV_MAT_DEPTH(type);
    const auto channel = CV_MAT_CN(type);
    CV_CheckType(type, depth == CV_8U && (channel == 1 || channel == 3),
                 "Only 8-bit grayscale or color images are supported");

    cv::Mat img;
    if (_img.channels() != 1) {
        cv::cvtColor(_img, img, cv::COLOR_BGR2GRAY);
    }
    else {
        img = _img.getMat();
    }

    image_u8_t im = {img.cols, img.rows, img.cols, img.data};

    zarray_t *detections = apriltag_detector_detect(d->_td, &im);
    const int nDetections = zarray_size(detections);

    std::vector<Detection> dets;
    for (int i{0}; i < nDetections; ++i) {
        apriltag_detection_t *detection;
        zarray_get(detections, i, &detection);

        Detection det;
        det.id = detection->id;
        det.hamming = detection->hamming;
        det.decisionMargin = detection->decision_margin;
        det.H = cv::Matx33d{detection->H->data};
        det.center = {detection->c[0], detection->c[1]};
        det.corners = {cv::Point2d{detection->p[0][0], detection->p[0][1]},
                       {detection->p[1][0], detection->p[1][1]},
                       {detection->p[2][0], detection->p[2][1]},
                       {detection->p[3][0], detection->p[3][1]}};

        dets.push_back(det);
    }

    // Destroy C style instances
    apriltag_detections_destroy(detections);

    if (_viz.needed()) {
        _img.copyTo(_viz);
        auto viz = _viz.getMat();

        for (const auto &det : dets) {
            const auto &corners = det.corners;

            // Draw box
            // cv::line(viz, corners[0], corners[1], cv::Scalar{0, 0xff, 0}, 2);
            // cv::line(viz, corners[0], corners[3], cv::Scalar{0, 0, 0xff}, 2);
            // cv::line(viz, corners[1], corners[2], cv::Scalar{0xff, 0, 0}, 2);
            // cv::line(viz, corners[2], corners[3], cv::Scalar{0xff, 0, 0}, 2);

            // Draw corners
            for (const auto &corner : corners) {
                if (!insideImage(corner, _img)) {
                    cv::drawMarker(viz, corner, CV_RGB(255, 0, 0),
                                   cv::MARKER_TILTED_CROSS, 5, 1, cv::LINE_AA);
                }
                else {
                    cv::drawMarker(viz, corner, CV_RGB(0, 255, 0),
                                   cv::MARKER_TILTED_CROSS, 5, 1, cv::LINE_AA);
                }
            }

            // Draw text
            // const auto text = std::to_string(det.id);
            // constexpr auto kFontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            // constexpr auto kFontScale = 1.;
            // constexpr auto kThickness = 2;
            // int baseline;
            // const auto textSize = cv::getTextSize(text, kFontFace,
            // kFontScale,
            //                                       kThickness, &baseline);
            // cv::putText(viz, text,
            //             det.center + cv::Point2d{-textSize.width / 2.,
            //                                      textSize.height / 2.},
            //             kFontFace, kFontScale, cv::Scalar{0xff, 0x99, 0},
            //             kThickness);
        }
    }

    return dets;
}

TagDetection AprilTag::detectPoints(cv::InputArray src,
                                    cv::OutputArray viz) const
{
    auto tagDetections = detect(src, viz);

    if (tagDetections.empty()) {
        return {};
    }

    std::ranges::sort(tagDetections,
                      [](const auto &a, const auto &b) { return a.id < b.id; });

    // Flatten detections
    TagDetection detection;
    detection.reserve(tagDetections.size());
    for (const auto &det : tagDetections) {
        const auto &tagId = det.id;
        auto bl_cornerId = tagId * 4;
        for (const auto &corner : det.corners) {
            const auto cornerId = bl_cornerId++;
            if (!insideImage(corner, src)) {
                continue;
            }

            detection.corners.emplace_back(corner.x, corner.y);
            detection.tagIds.emplace_back(tagId);
            detection.cornerIds.emplace_back(cornerId);
        }
    }

    return detection;
}

AprilTag::Board::Detections AprilTag::detectBoards(
    cv::InputArray src, const std::vector<Board::Options> &options,
    cv::OutputArray viz) const
{
    auto tagDetections = detect(src, viz);
    if (tagDetections.empty()) {
        return {};
    }

    std::ranges::sort(tagDetections,
                      [](const auto &a, const auto &b) { return a.id < b.id; });

    std::vector<int> tagIds;
    tagIds.reserve(tagDetections.size());
    std::ranges::transform(tagDetections, std::back_inserter(tagIds),
                           [](const auto &detection) { return detection.id; });

    ///        Corner ordering in board points :
    ///          12-----13  14-----15
    ///          | TAG 3 |  | TAG 4 |
    ///          8-------9  10-----11
    ///          4-------5  6-------7
    ///    y     | TAG 1 |  | TAG 2 |
    ///   ^      0-------1  2-------3
    ///   |-->x
    ///
    Board::Detections boardDetections;
    boardDetections.reserve(options.size());
    for (const auto &opts : options) {
        const auto &startTagId = opts.startTagId;
        const auto startCornerId = opts.startTagId * 4;
        const auto cornerWidth = opts.tagDim.width * 2;

        const auto minTagId_it = std::ranges::lower_bound(tagIds, startTagId);
        const auto maxTagId_it = std::ranges::upper_bound(
            tagIds, startTagId + opts.tagDim.area() - 1);
        const std::vector<int> tagIdsOnBoard{minTagId_it, maxTagId_it};

        if (tagIdsOnBoard.empty()) {
            boardDetections.push_back({});
            continue;
        }

        const auto startTag_i = std::distance(tagIds.begin(), minTagId_it);

        Board::Detection boardDetection;
        boardDetection.reserve(tagIdsOnBoard.size() * 4);
        for (size_t i{0}; i < tagIdsOnBoard.size(); ++i) {
            const auto &tagId = tagIdsOnBoard[i];
            const auto &tagDetection = tagDetections[startTag_i + i];

            const auto tag_dist = tagId - startTagId;
            const auto tag_x = tag_dist % opts.tagDim.width;
            const auto tag_y = tag_dist / opts.tagDim.width;

            const auto bl_id =
                startCornerId + tag_x * 2 + tag_y * 2 * cornerWidth;

            ///  2<-------3
            ///  |        +
            ///  +  TAG   |
            ///  0------->1
            ///
            // clang-format off
            boardDetection.cornerIds.insert(
                boardDetection.cornerIds.end(),
                {bl_id,
                 bl_id + 1,
                 bl_id + cornerWidth + 1,
                 bl_id + cornerWidth});
            // clang-format on

            for (const auto &corner : tagDetection.corners) {
                boardDetection.corners.push_back(corner);
            }
        }

        boardDetections.push_back(boardDetection);
    }

    return boardDetections;
}

} // namespace tl
