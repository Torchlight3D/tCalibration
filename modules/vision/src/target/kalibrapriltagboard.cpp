#include "kalibrapriltagboard.h"

#include <numeric>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "apriltag_mit/TagDetector.h"
// #include "codec/Tag16h5.h"
// #include "codec/Tag25h7.h"
// #include "codec/Tag25h9.h"
// #include "codec/Tag36h9.h"
#include "codec/Tag36h11.h"

namespace tl {

using Eigen::Vector2d;

struct EdgeCostFunctor
{
    cv::Mat raw_img;
    cv::Mat mask_img;
    cv::Mat template_img;

    EdgeCostFunctor(const cv::Mat &rawImg, const cv::Mat &maskImg)
        : raw_img(rawImg), mask_img(maskImg)
    {
        template_img.create(1, 2, CV_8U);
        template_img.data[0] = 0;
        template_img.data[1] = 255;
    }

    template <typename T>
    T getSubPixel(const cv::Mat &img, cv::Point_<T> pt) const
    {
        const T x = floor(pt.x);
        const T y = floor(pt.y);

        const int x0 = cv::borderInterpolate(x, img.cols, cv::BORDER_REPLICATE);
        const int x1 =
            cv::borderInterpolate(x + 1, img.cols, cv::BORDER_REPLICATE);
        const int y0 = cv::borderInterpolate(y, img.rows, cv::BORDER_REPLICATE);
        const int y1 =
            cv::borderInterpolate(y + 1, img.rows, cv::BORDER_REPLICATE);

        const T a = pt.x - T(x);
        const T c = pt.y - T(y);

        return (T(img.at<uint8_t>(y0, x0)) * (T(1) - a) +
                T(img.at<uint8_t>(y0, x1)) * a) *
                   (T(1) - c) +
               (T(img.at<uint8_t>(y1, x0)) * (T(1) - a) +
                T(img.at<uint8_t>(y1, x1)) * a) *
                   c;
    }

    template <typename T>
    bool operator()(const T *const _edgeNormal, const T *const _edgeOffset,
                    T *residual) const
    {
        const Eigen::Map<const Eigen::Vector2<T>> edgeNormal(_edgeNormal);
        const auto &edgeOffset = *_edgeOffset;

        const Eigen::Hyperplane<T, 2> line{edgeNormal, edgeOffset};

        int residual_index = 0;
        for (int row = 0; row < raw_img.rows; row++) {
            const auto raw_img_row_ptr = raw_img.ptr<uint8_t>(row);
            const auto mask_row_ptr = mask_img.ptr<uint8_t>(row);

            for (int col = 0; col < raw_img.cols; col++) {
                const auto eval = mask_row_ptr[col];

                if (eval) {
                    const Vector2d raw_img_pos(col, row);
                    const double dist_to_line =
                        line.signedDistance(raw_img_pos);

                    const cv::Point2d template_pt(dist_to_line + 0.5, 0);

                    const double pred_pixel_value =
                        getSubPixel<double>(template_img, template_pt);
                    const uint8_t current_pixel_values = raw_img_row_ptr[col];
                    residual[residual_index] =
                        pred_pixel_value - current_pixel_values;
                    residual_index++;
                }
            }
        }

        return true;
    }
};

void refineCornerPointsByDirectEdgeOptimization(
    cv::Mat &img, std::vector<AprilTags::TagDetection> &detections)
{
    for (auto &detection : detections) {
        const auto bbox = cv::boundingRect(detection.p);
        const cv::Rect validArea{cv::Point{10, 10},
                                 cv::Point{img.cols - 1, img.rows - 1}};

        try {
            const auto roi = bbox & validArea;

            cv::Mat cropped_img = img(roi);

            std::array<Vector2d, 4> estimated_edge_normals;
            std::array<double, 4> estimated_edge_offsets;
            std::array<cv::Mat, 4> mask_images;

            int line_thickness = 5;

            auto nextCornerIndex = [](int i) { return (i + 1) % 4; };

            for (auto i{0}; i < 4; i++) {
                const int j = nextCornerIndex(i);

                const Vector2d roi_offset_vector(roi.x, roi.y);

                const Vector2d corner_i(detection.p[i].x, detection.p[i].y);
                const Vector2d corner_j(detection.p[j].x, detection.p[j].y);

                const auto edge_line = Eigen::Hyperplane<double, 2>::Through(
                    corner_i - roi_offset_vector, corner_j - roi_offset_vector);

                estimated_edge_normals[i] = edge_line.normal();
                estimated_edge_offsets[i] = edge_line.offset();

                mask_images[i].create(cropped_img.rows, cropped_img.cols,
                                      CV_8U);
                mask_images[i].setTo(0);

                const cv::Point current_corner_point =
                    detection.p[i] - cv::Point2f(roi.tl());
                const cv::Point next_corner_point =
                    detection.p[j] - cv::Point2f(roi.tl());

                cv::line(mask_images[i], current_corner_point,
                         next_corner_point, cv::Scalar(255), line_thickness);
                cv::rectangle(mask_images[i], current_corner_point,
                              current_corner_point, cv::Scalar(0), 10);
                cv::rectangle(mask_images[i], next_corner_point,
                              next_corner_point, cv::Scalar(0), 10);
            }

            ceres::Problem problem;
            ceres::NumericDiffOptions numeric_diff_options;

            auto addEdgeResidualBlocks = [&problem, &mask_images, &cropped_img,
                                          &estimated_edge_normals,
                                          &estimated_edge_offsets,
                                          &numeric_diff_options,
                                          &nextCornerIndex](int i) {
                const int pixel_count = cv::countNonZero(mask_images[i]);

                auto *cost = new ceres::NumericDiffCostFunction<
                    EdgeCostFunctor, ceres::CENTRAL, ceres::DYNAMIC, 2, 1>(
                    new EdgeCostFunctor(cropped_img, mask_images[i]),
                    ceres::TAKE_OWNERSHIP, pixel_count, numeric_diff_options);
                problem.AddResidualBlock(cost, nullptr,
                                         estimated_edge_normals[i].data(),
                                         &estimated_edge_offsets[i]);

                problem.SetManifold(
                    estimated_edge_normals[i].data(),
                    new ceres::SphereManifold<ceres::DYNAMIC>(2));
            };

            addEdgeResidualBlocks(0);
            addEdgeResidualBlocks(1);
            addEdgeResidualBlocks(2);
            addEdgeResidualBlocks(3);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 100;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            for (auto i{0}; i < 4; i++) {
                const int j = nextCornerIndex(i);

                const Eigen::Hyperplane<double, 2> edge_A(
                    estimated_edge_normals[i], estimated_edge_offsets[i]);
                const Eigen::Hyperplane<double, 2> edge_B(
                    estimated_edge_normals[j], estimated_edge_offsets[j]);

                const Vector2d estimated_corner_pos_roi =
                    edge_A.intersection(edge_B);

                detection.p[j].x = estimated_corner_pos_roi.x() + roi.x;
                detection.p[j].y = estimated_corner_pos_roi.y() + roi.y;
            }
        }
        catch (const std::exception & /*e*/) {
            detection.good = false;
        }
    }

    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
                       [](const auto &detection) { return !detection.good; }),
        detections.end());
}

class KalibrAprilTagBoard::Impl
{
public:
    Impl();

    void init(const Options &opts, const DetectOptions &detectOpts);

public:
    Options _opts;
    DetectOptions _detectOpts;

    // Apriltag
    tl::TagCodes _tagCodes = tl::tagCodes36h11;
    std::unique_ptr<AprilTags::TagDetector> _tagDetector;
};

KalibrAprilTagBoard::Impl::Impl() {}

void KalibrAprilTagBoard::Impl::init(const Options &opts,
                                     const DetectOptions &detectOpts)
{
    // Board
    _opts = opts;
    _tagCodes = tl::tagCodes36h11;

    // Detector
    _detectOpts = detectOpts;
    _tagDetector =
        std::make_unique<AprilTags::TagDetector>(_tagCodes, opts.blackBorder);
}

/// \brief Construct an Aprilgrid calibration target
///        tagRows:    number of tags in y-dir (gridRows = 2*tagRows)
///        tagCols:    number of tags in x-dir (gridCols = 2*tagCols)
///        tagSize:    size of a tag [m]
///        tagSpacing: space between tags (in tagSpacing [m] =
///        tagSpacing*tagSize)
///
///        corner ordering in _points :
///          12-----13  14-----15
///          | TAG 3 |  | TAG 4 |
///          8-------9  10-----11
///          4-------5  6-------7
///    y     | TAG 1 |  | TAG 2 |
///   ^      0-------1  2-------3
///   |-->x
KalibrAprilTagBoard::KalibrAprilTagBoard(const Options &opts,
                                         const DetectOptions &detectOpts)
    : CalibBoardBase(2 * opts.tagRows, 2 * opts.tagCols),
      d(std::make_unique<Impl>())
{
    d->init(opts, detectOpts);

    createBoardPoints();
}

KalibrAprilTagBoard::~KalibrAprilTagBoard() = default;

void KalibrAprilTagBoard::createBoardPoints()
{
    m_boardPoints.clear();

    const auto tagWithSpace = (1 + d->_opts.tagSpacingRatio) * d->_opts.tagSize;
    for (int r{0}; r < m_rows; r++) {
        for (int c{0}; c < m_cols; c++) {
            m_boardPoints.emplace_back(
                (c / 2) * tagWithSpace + (c % 2) * d->_opts.tagSize,
                (r / 2) * tagWithSpace + (r % 2) * d->_opts.tagSize, 0.);
        }
    }

    std::iota(m_ids.begin(), m_ids.end(), 0);
}

double KalibrAprilTagBoard::tagDistance() const
{
    return d->_opts.tagSpacingRatio * d->_opts.tagSize;
}

double KalibrAprilTagBoard::tagSize() const { return d->_opts.tagSize; }

TargetDetection KalibrAprilTagBoard::computeObservation(
    cv::InputArray _img, cv::OutputArray _viz) const
{
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

    // 1. Detect the tags
    auto detections = d->_tagDetector->extractTags(img);
    if (detections.empty()) {
        return {};
    }

    // 2. Postprocessing
    // Rule.1
    // Tag is identified but not all the corners are in the image. This
    // situation also includes any encoded pattern (data bits) is in the view,
    // but not its border. These corners may still be fine, as apriltag lib will
    // extrapolate them, but it could fail the following subpix refinement.
    const cv::Vec2f border{d->_detectOpts.minBorderDistance,
                           d->_detectOpts.minBorderDistance};
    const cv::Rect2f inBorderROI{
        cv::Point2f{border}, cv::Size2f(img.size()) - cv::Size2f(border * 2)};
    for (auto it = detections.begin(); it != detections.end();) {
        bool remove = false;

        // Check if all four corners in image
        for (const auto &corner : it->p) {
            remove |= (!inBorderROI.contains(corner));
        }

        // Check if the tag is flagged as bad
        remove |= (!it->good);

        // Check if the tag ID belongs to this board
        remove |= (it->id < d->_opts.startId) ||
                  (it->id >= cornerCount() / 4 + d->_opts.startId);

        if (remove) {
            LOG(INFO) << "Tag " << it->id << " is removed: "
                      << "It is partially seen, "
                         "or it doesn't belong to this board.";
            it = detections.erase(it);
        }
        else {
            ++it;
        }
    }

    // Optional: Check if enough valid tags are detected.
    if (detections.size() < d->_detectOpts.minDetectTagCount) {
        return {};
    }

    std::sort(detections.begin(), detections.end(),
              AprilTags::TagDetection::sortByIdCompare);

    // Check if there are duplicated tags exist.
    if (detections.size() > 1) {
        if (const auto found =
                std::adjacent_find(detections.cbegin(), detections.cend(),
                                   [](const auto &curr, const auto &next) {
                                       return curr.id == next.id;
                                   });
            found != detections.cend()) {
            LOG(ERROR) << "Detection failed: "
                          "Found duplicated tag.";
            return {};
        }
    }

    // Convert corners to cv::Mat (4 consecutive corners form one tag)
    /// point ordering here
    ///          11-----10  15-----14
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  12-----13
    ///          3-------2  7-------6
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  4-------5
    ///   |-->x
    std::vector<cv::Point2f> tagCorners;
    tagCorners.reserve(4 * detections.size());
    for (const auto &detection : detections) {
        for (const auto &pt : detection.p) {
            tagCorners.push_back(pt);
        }
    }

    // Optional: Subpixel refinement on all tag corners (four corners each tag)
    auto tagCornersRefined = tagCorners;
    if (d->_detectOpts.doSubpixRefinement) {
        cv::cornerSubPix(img, tagCornersRefined, cv::Size{2, 2},
                         cv::Size{-1, -1},
                         cv::TermCriteria{cv::TermCriteria::Type::EPS +
                                              cv::TermCriteria::Type::MAX_ITER,
                                          30, 0.1});
    }

    // Insert the observed points into the correct location of the grid point
    // array
    /// point ordering
    ///          12-----13  14-----15
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  10-----11
    ///          4-------5  6-------7
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  2-------3
    ///   |-->x
    std::vector<CornerId> cornerIds(cornerCount(), -1);
    std::vector<cv::Point2d> corners(cornerCount(), cv::Point2d{});
    std::vector<uchar> detected(cornerCount(), 0);
    auto count{0};
    for (size_t i{0}; i < detections.size(); i++) {
        unsigned int tagId = detections[i].id - d->_opts.startId;
        unsigned int tagblCornerId = (int)(tagId / (cols() / 2)) * cols() * 2 +
                                     (tagId % (cols() / 2)) * 2;
        unsigned int tagCornerIds[] = {tagblCornerId, tagblCornerId + 1,
                                       tagblCornerId + (unsigned int)cols() + 1,
                                       tagblCornerId + (unsigned int)cols()};

        // Per tag
        for (size_t j{0}; j < 4; j++) {
            const auto idx = 4 * i + j;
            const auto &thisCornerId = tagCornerIds[j];

            const auto &corner = tagCorners[idx];
            const auto &cornerRefined = tagCornersRefined[idx];
            const auto diff = cv::norm(cornerRefined - corner);

            cornerIds[thisCornerId] = thisCornerId;
            corners[thisCornerId] = cornerRefined;
            if (diff <= d->_detectOpts.maxSubpixDisplacement) {
                detected[thisCornerId] = 1;
                ++count;
            }
            else {
                LOG(INFO) << "Subpix refinement failed for point "
                          << thisCornerId << " with displacement: " << diff
                          << "(point removed)";
                detected[thisCornerId] = 0;
            }
        }
    }

    // Optional: Check overall detect rate
    if (const int minDetectCount = d->_detectOpts.minDetectRate * cornerCount();
        count < minDetectCount) {
        return {};
    }

    if (_viz.needed()) {
        _img.copyTo(_viz);
        cv::Mat viz;
        if (channel == 1) {
            cv::cvtColor(_viz, viz, cv::COLOR_GRAY2BGR);
        }
        else {
            viz = _viz.getMat();
        }

        for (const auto &detection : detections) {
            detection.draw(viz);
        }
    }

    return {.corners = corners,
            .cornerIds = cornerIds,
            .detected = detected,
            .count = count};
}

namespace key {
constexpr char kTagRows[]{"tag_rows"};
constexpr char kTagCols[]{"tag_cols"};
constexpr char kTagSize[]{"tag_size"};
constexpr char kTagSpacingRatio[]{"tag_spacing_ratio"};
constexpr char kBlackBorder[]{"black_border"};
constexpr char kStartId[]{"start_id"};
// ApriltagBoard
constexpr char kMaxSubPixDiff[]{"max_subpix_diff"};
constexpr char kMinDetectTagCount[]{"min_detect_tag_count"};
constexpr char kMinDetectRate[]{"min_detect_rate"};
constexpr char kMinBorderDistance[]{"min_border_distance"};

} // namespace key

void to_json(nlohmann::json &j, const KalibrAprilTagBoard::Options &opts)
{
    j[key::kTagRows] = opts.tagRows;
    j[key::kTagCols] = opts.tagCols;
    j[key::kTagSize] = opts.tagSize;
    j[key::kTagSpacingRatio] = opts.tagSpacingRatio;
    j[key::kBlackBorder] = opts.blackBorder;
    j[key::kStartId] = opts.startId;
}

void from_json(const nlohmann::json &j, KalibrAprilTagBoard::Options &opts)
{
    j.at(key::kTagRows).get_to(opts.tagRows);
    j.at(key::kTagCols).get_to(opts.tagCols);
    j.at(key::kTagSize).get_to(opts.tagSize);
    j.at(key::kTagSpacingRatio).get_to(opts.tagSpacingRatio);
    j.at(key::kBlackBorder).get_to(opts.blackBorder);
    j.at(key::kStartId).get_to(opts.startId);
}

void to_json(nlohmann::json &j, const KalibrAprilTagBoard::DetectOptions &opts)
{
    if (opts.doSubpixRefinement) {
        j[key::kMaxSubPixDiff] = opts.maxSubpixDisplacement;
    }
    else {
        j[key::kMaxSubPixDiff] = nullptr;
    }
    j[key::kMinDetectTagCount] = opts.minDetectTagCount;
    j[key::kMinDetectRate] = opts.minDetectRate;
    j[key::kMinBorderDistance] = opts.minBorderDistance;
}

void from_json(const nlohmann::json &j,
               KalibrAprilTagBoard::DetectOptions &opts)
{
    if (const auto val = j.at(key::kMaxSubPixDiff); val.is_null()) {
        opts.doSubpixRefinement = false;
        opts.maxSubpixDisplacement = std::numeric_limits<double>::max();
    }
    else {
        opts.doSubpixRefinement = true;
        j.at(key::kMaxSubPixDiff).get_to(opts.maxSubpixDisplacement);
    }
    j.at(key::kMinDetectTagCount).get_to(opts.minDetectTagCount);
    j.at(key::kMinDetectRate).get_to(opts.minDetectRate);
    j.at(key::kMinBorderDistance).get_to(opts.minBorderDistance);
}

} // namespace tl
