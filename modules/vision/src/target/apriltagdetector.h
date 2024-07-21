#pragma once

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

namespace tl {

// TODO: Promote to higher scope
struct TagDetection
{
    // Corners
    std::vector<Eigen::Vector2d> corners;

    // Corner Ids
    std::vector<int> cornerIds;

    // Tag Ids
    std::vector<int> tagIds;

    void reserve(size_t size)
    {
        const auto cornerCount = size * 4;
        corners.reserve(cornerCount);
        cornerIds.reserve(cornerCount);
        tagIds.reserve(cornerCount);
    }

    size_t size() const { return corners.size(); }

    bool empty() const { return corners.empty(); }
};

class AprilTag
{
public:
    enum Family
    {
        tag16h5,
        tag25h9,
        tag36h11,
        tag36h11b2,
        tagCircle21h7,
        tagCircle49h12,
        tagStandard41h12,
        tagStandard52h13,
        tagCustom48h12,
    };

    struct Options
    {
        // TagFamilly to use
        Family family = Family::tag36h11;

        // Sharpening extent of decoded images.
        // This can help decode small tags but may or may not help in odd
        // lighting conditions or low light conditions.
        double sharpening = 0.25;

        // Decimate input image by this factor
        float decimate = 2.f;

        // Apply low pass blur to input image
        float blur = 0.f;

        // Threads to use
        int thread = 1;

        // Try to align edges of tags
        // NOTE: If refineEdges is enabled, some of the detected corners may
        // locate outside the image.
        bool refineEdges = false;

        Options() {}
    };

    explicit AprilTag(const Options& opts = {});
    ~AprilTag();

    void setOptions(const Options& opts);

    struct Detection
    {
        // The detected tag Id
        int id;

        // Error bits corrected.
        // NOTE: Accep large numbers of corrected errors leads to increased
        // false positive rates.
        int hamming = 2;

        // Quality of the binary decoding process. The higher, the better
        // decode.
        float decisionMargin;

        // The homography matrix describing the projection from ideal tag
        // Rect{Point{-1, 1}, Point{1, -1}} to the pixels in the image.
        cv::Matx33d H;

        cv::Point2d center;
        std::array<cv::Point2d, 4> corners;
    };

    std::vector<Detection> detect(cv::InputArray src,
                                  cv::OutputArray viz = cv::noArray()) const;

    // [Overload]: Return the flattened tag detections
    TagDetection detectPoints(cv::InputArray src,
                              cv::OutputArray viz = cv::noArray()) const;

    struct Board
    {
        std::vector<cv::Point3f> points;
        std::vector<int> ids;

        struct Options
        {
            // Numbers of tag in row and column
            cv::Size tagDim;

            // Tag size in meter
            float tagSize = 0.1f;

            // Tag spacing as the ratio to tag size
            float tagSpacingRatio = 0.3f;

            // Assume tag Ids on the board are in ascending order from
            // startTagId
            int startTagId = 0;

            bool isValid() const;
        };

        explicit Board(const Options& opts);

        bool isValid() const;

        // This method would sort the options by ascending start tag id
        static bool checkGroupValid(std::vector<Options>& options);

        struct Detection
        {
            std::vector<cv::Point2f> corners;
            std::vector<int> cornerIds;

            void reserve(size_t cap);
            size_t size() const;
            bool empty() const;
        };
        using Detections = std::vector<Detection>;
    };

    Board::Detections detectBoards(cv::InputArray src,
                                   const std::vector<Board::Options>& options,
                                   cv::OutputArray viz = cv::noArray()) const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;
};

} // namespace tl
