#pragma once

#include <memory>

#include "calib_board_base.h"

// April tags detector and various tag families
#include "apriltag_mit/TagDetector.h"
// #include "apriltag_mit/Tag16h5.h"
// #include "apriltag_mit/Tag25h7.h"
// #include "apriltag_mit/Tag25h9.h"
// #include "apriltag_mit/Tag36h9.h"
#include "apriltag_mit/Tag36h11.h"

namespace tl {

// Brief:
// AprilTagBoard from Kalibr, for comparison purpose. DONT change any important
// logic or strategy.
class KalibrAprilTagBoard : public CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<KalibrAprilTagBoard>;
    using ConstPtr = std::shared_ptr<const KalibrAprilTagBoard>;

    // target extraction options
    struct AprilgridOptions
    {
        AprilgridOptions()
            : doSubpixRefinement(true),
              maxSubpixDisplacement2(1.5),
              showExtractionVideo(false),
              minTagsForValidObs(4),
              minBorderDistance(4.0),
              blackTagBorder(2)
        {
        }

        // options
        /// \brief subpixel refinement of extracted corners
        bool doSubpixRefinement;

        /// \brief max. displacement squarred in subpixel refinement  [px^2]
        double maxSubpixDisplacement2;

        /// \brief show video during extraction
        bool showExtractionVideo;

        /// \brief min. number of tags for a valid observation
        unsigned int minTagsForValidObs;

        /// \brief min. distance form image border for valid points [px]
        double minBorderDistance;

        /// \brief size of black border around the tag code bits (in pixels)
        unsigned int blackTagBorder;
    };

    // NOTE: tagSpacing is a ratio!!!
    /// \brief initialize based on checkerboard geometry
    KalibrAprilTagBoard(size_t tagRows, size_t tagCols, double tagSize,
                        double tagSpacing,
                        const AprilgridOptions &options = AprilgridOptions());

    double tagDistance() const override;
    double tagSize() const override;

    /// \brief extract the calibration target points from an image and write to
    /// an observation
    TargetDetection computeObservation(cv::InputArray image) const override;

    void drawDetection(const TargetDetection &detection,
                       cv::Mat &imgResult) const override;

private:
    /// \brief initialize the grid with the points
    void createBoardPoints() override;

    /// \brief size of a tag [m]
    double _tagSize;

    /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
    double _tagSpacing;

    /// \brief target extraction options
    AprilgridOptions _options;

    // create a detector instance
    AprilTags::TagCodes _tagCodes;
    std::shared_ptr<AprilTags::TagDetector> _tagDetector;
    mutable std::vector<AprilTags::TagDetection> _lastDetections;
};

} // namespace tl
