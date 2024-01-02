#include "kalibrapriltagboard.h"

#include <numeric>
#include <glog/logging.h>

namespace thoht {

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
KalibrAprilTagBoard::KalibrAprilTagBoard(size_t tagRows, size_t tagCols,
                                         double tagSize, double tagSpacing,
                                         const AprilgridOptions &options)
    : CalibBoardBase(2 * tagRows, 2 * tagCols), // 4 points per tag
      _tagSize(tagSize),
      _tagSpacing(tagSpacing),
      _options(options),
      _tagCodes(AprilTags::tagCodes36h11)
{
    // initialize a normal grid (checkerboard and circlegrids)
    createBoardPoints();

    // create the tag detector
    _tagDetector = std::make_shared<AprilTags::TagDetector>(
        _tagCodes, _options.blackTagBorder);
}

/// \brief initialize an april grid
///   point ordering: (e.g. 2x2 grid)
///          12-----13  14-----15
///          | TAG 3 |  | TAG 4 |
///          8-------9  10-----11
///          4-------5  6-------7
///    y     | TAG 1 |  | TAG 2 |
///   ^      0-------1  2-------3
///   |-->x
void KalibrAprilTagBoard::createBoardPoints()
{
    // each tag has 4 corners
    // unsigned int numTags = size()/4;
    // unsigned int colsTags = _cols/2;

    for (unsigned r = 0; r < m_rows; r++) {
        for (unsigned c = 0; c < m_cols; c++) {
            m_boardPoints.emplace_back(
                (int)(c / 2) * (1 + _tagSpacing) * _tagSize +
                    (c % 2) * _tagSize,
                (int)(r / 2) * (1 + _tagSpacing) * _tagSize +
                    (r % 2) * _tagSize,
                0.);
        }
    }

    std::iota(m_ids.begin(), m_ids.end(), 0);
}

double KalibrAprilTagBoard::tagDistance() const { return _tagSpacing; }

double KalibrAprilTagBoard::tagSize() const { return _tagSize; }

/// \brief extract the calibration target points from an image and write to an
/// observation
TargetDetection KalibrAprilTagBoard::computeObservation(
    cv::InputArray image) const
{
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

    bool success = true;

    // Detect the tags
    _lastDetections = _tagDetector->extractTags(img);
    if (_lastDetections.empty()) {
        return {};
    }

    /* handle the case in which a tag is identified but not all tag
     * corners are in the image (all data bits in image but border
     * outside). tagCorners should still be okay as apriltag-lib
     * extrapolates them, only the subpix refinement will fail
     */

    // min. distance [px] of tag corners from image border (tag is not used if
    // violated)
    for (auto it = _lastDetections.begin(); it != _lastDetections.end();) {
        // check all four corners for violation
        bool remove = false;

        for (int j = 0; j < 4; j++) {
            remove |= it->p[j].first < _options.minBorderDistance;
            remove |= it->p[j].first >
                      (float)(img.cols) - _options.minBorderDistance; // width
            remove |= it->p[j].second < _options.minBorderDistance;
            remove |= it->p[j].second >
                      (float)(img.rows) - _options.minBorderDistance; // height
        }

        // also remove tags that are flagged as bad
        if (it->good != 1)
            remove |= true;

        // also remove if the tag ID is out-of-range for this grid (faulty
        // detection)
        if (it->id >= cornerCount() / 4)
            remove |= true;

        // delete flagged tags
        if (remove) {
            LOG(INFO)
                << "Tag with ID " << it->id
                << " is only partially in image (corners outside) and will be "
                   "removed from the TargetObservation.";

            // delete the tag and advance in list
            it = _lastDetections.erase(it);
        }
        else {
            // advance in list
            ++it;
        }
    }

    // did we find enough tags?
    if (_lastDetections.size() < _options.minTagsForValidObs) {
        success = false;

        // immediate exit if we dont need to show video for debugging...
        // if video is shown, exit after drawing video...
        if (!_options.showExtractionVideo) {
            return {};
        }
    }

    // sort detections by tagId
    std::sort(_lastDetections.begin(), _lastDetections.end(),
              AprilTags::TagDetection::sortByIdCompare);

    // check for duplicate tagIds (--> if found: wild Apriltags in image not
    // belonging to calibration target) (only if we have more than 1 tag...)
    if (_lastDetections.size() > 1) {
        for (size_t i = 0; i < _lastDetections.size() - 1; i++)
            if (_lastDetections[i].id == _lastDetections[i + 1].id) {
                // and exit
                LOG(ERROR) << "Found apriltag not belonging to calibration "
                              "board. Check the image for the tag and hide it.";
                return {};
            }
    }

    // convert corners to cv::Mat (4 consecutive corners form one tag)
    /// point ordering here
    ///          11-----10  15-----14
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  12-----13
    ///          3-------2  7-------6
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  4-------5
    ///   |-->x
    cv::Mat tagCorners(4 * _lastDetections.size(), 2, CV_32F);

    for (size_t i = 0; i < _lastDetections.size(); i++) {
        for (int j = 0; j < 4; j++) {
            const int row = 4 * i + j;
            tagCorners.at<float>(row, 0) = _lastDetections[i].p[j].first;
            tagCorners.at<float>(row, 1) = _lastDetections[i].p[j].second;
        }
    }

    // store a copy of the corner list before subpix refinement
    cv::Mat tagCornersRaw = tagCorners.clone();

    // optional subpixel refinement on all tag corners (four corners each tag)
    if (_options.doSubpixRefinement && success) {
        cv::cornerSubPix(img, tagCorners, cv::Size(2, 2), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::Type::EPS +
                                              cv::TermCriteria::Type::MAX_ITER,
                                          30, 0.1));
    }

    // insert the observed points into the correct location of the grid point
    // array
    /// point ordering
    ///          12-----13  14-----15
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  10-----11
    ///          4-------5  6-------7
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  2-------3
    ///   |-->x

    std::vector<CornerId> cornerIds;
    cornerIds.reserve(cornerCount());
    std::vector<cv::Point2d> corners;
    corners.reserve(cornerCount());

    for (size_t i = 0; i < _lastDetections.size(); i++) {
        unsigned int tagId = _lastDetections[i].id;

        // calculate the grid idx for all four tag corners given the tagId and
        // cols
        unsigned int baseId = (int)(tagId / (cols() / 2)) * cols() * 2 +
                              (tagId % (cols() / 2)) * 2;
        unsigned int pIdx[] = {baseId, baseId + 1,
                               baseId + (unsigned int)cols() + 1,
                               baseId + (unsigned int)cols()};

        // add four points per tag
        for (int j = 0; j < 4; j++) {
            const int row = 4 * i + j;

            auto refinedCorner_x = tagCorners.at<float>(row, 0);
            auto refinedCorner_y = tagCorners.at<float>(row, 1);
            auto rawCorner_x = tagCornersRaw.at<float>(row, 0);
            auto rawCorner_y = tagCornersRaw.at<float>(row, 1);

            // only add point if the displacement in the subpixel refinement is
            // below a given threshold
            double refineDisplacement = (refinedCorner_x - rawCorner_x) *
                                            (refinedCorner_x - rawCorner_x) +
                                        (refinedCorner_y - rawCorner_y) *
                                            (refinedCorner_y - rawCorner_y);

            // add all points, but only set active if the point has not moved to
            // far in the subpix refinement
            // NOTE: No, that's not what happens
            if (refineDisplacement <= _options.maxSubpixDisplacement2) {
                cornerIds.emplace_back(pIdx[j]);
                corners.emplace_back(refinedCorner_x, refinedCorner_y);
            }
            else {
                LOG(INFO) << "Subpix refinement failed for point " << pIdx[j]
                          << " with displacement: " << sqrt(refineDisplacement)
                          << "(point removed)";
            }
        }
    }

    return {.cornerIds = cornerIds, .corners = corners, .valid = true};
}

void KalibrAprilTagBoard::drawDetection(const TargetDetection &detection,
                                        cv::Mat &imgResult) const
{
    for (const auto &detection : _lastDetections) {
        detection.draw(imgResult);
    }
}

} // namespace thoht
