#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {
namespace stag {

/**
 * Detects markers in given image. Maximum possible error correction for the
 * given libraryHD is used.
 * @param inImage OpenCV Matrix of input image.
 * @param libraryHD The library HD that is used. Possible values are
 * [11,&nbsp;13,&nbsp;15,&nbsp;17,&nbsp;19,&nbsp;21,&nbsp;23].
 * @param output_corners The vector where the corners of detected markers are
 * written into.
 * @param output_ids The vector where the ids of detected markers are written
 * into.
 */
void detectMarkers(const cv::Mat& image, int libraryHD,
                   std::vector<std::vector<cv::Point2f>>& output_corners,
                   std::vector<int>& output_ids);

/**
 * Detects markers in given image.
 * @param inImage OpenCV Matrix of input image.
 * @param libraryHD The library HD that is used. Possible values are
 * [11,&nbsp;13,&nbsp;15,&nbsp;17,&nbsp;19,&nbsp;21,&nbsp;23].
 * @param output_corners The vector where the corners of detected markers are
 * written into.
 * @param output_ids The vector where the ids of detected markers are written
 * into.
 * @param errorCorrection The amount of error correction that is going to be
 * used. Value needs to be in range
 * 0&nbsp;\<=&nbsp;errorCorrection&nbsp;\<=&nbsp;(HD-1)/2.\n If set to -1, the
 * max possible value for the given library HD is used.
 */
void detectMarkers(const cv::Mat& image, int libraryHD,
                   std::vector<std::vector<cv::Point2f>>& output_corners,
                   std::vector<int>& output_ids, int errorCorrection);

/**
 * Detects markers in given image. Maximum possible error correction for the
 * given libraryHD is used.
 * @param inImage OpenCV Matrix of input image.
 * @param libraryHD The library HD that is used. Possible values are
 * [11,&nbsp;13,&nbsp;15,&nbsp;17,&nbsp;19,&nbsp;21,&nbsp;23].
 * @param output_corners The vector where the corners of detected markers are
 * written into.
 * @param output_ids The vector where the ids of detected markers are written
 * into.
 * @param output_rejectedImgPoints The vector where the corners of rejected
 * squares are written into.
 */
void detectMarkers(
    const cv::Mat& image, int libraryHD,
    std::vector<std::vector<cv::Point2f>>& output_corners,
    std::vector<int>& output_ids,
    std::vector<std::vector<cv::Point2f>>& output_rejectedImgPoints);

/**
 * Detects markers in given image.
 * @param inImage OpenCV Matrix of input image.
 * @param libraryHD The library HD that is used. Possible values are
 * [11,&nbsp;13,&nbsp;15,&nbsp;17,&nbsp;19,&nbsp;21,&nbsp;23].
 * @param output_corners The vector where the corners of detected markers are
 * written into.
 * @param output_ids The vector where the ids of detected markers are written
 * into.
 * @param errorCorrection The amount of error correction that is going to be
 * used. Value needs to be in range
 * 0&nbsp;\<=&nbsp;errorCorrection&nbsp;\<=&nbsp;(HD-1)/2.\n If set to -1, the
 * max possible value for the given library HD is used.
 * @param output_rejectedImgPoints The vector where the corners of rejected
 * squares are written into.
 */
void detectMarkers(
    const cv::Mat& image, int libraryHD,
    std::vector<std::vector<cv::Point2f>>& output_corners,
    std::vector<int>& output_ids, int errorCorrection,
    std::vector<std::vector<cv::Point2f>>& output_rejectedImgPoints);

/**
 * Draw detected markers in image
 * @param image Input/output image. It must have 1 or 3 channels. The number of
 * channels is not altered.
 * @param corners Position of marker corners on input image. For N detected
 * markers, the dimensions of this array should be Nx4.
 * @param ids Vector of identifiers for markers in param @corners. If omitted or
 * empty, no ids are drawn.
 * @param borderColor Color of marker borders.
 *
 * @throws invalid_argument If number of ids ist not zero and does not match
 * number of markers.
 */
void drawDetectedMarkers(cv::Mat& image,
                         const std::vector<std::vector<cv::Point2f>>& corners,
                         const std::vector<int>& ids = std::vector<int>(),
                         const cv::Scalar& borderColor = cv::Scalar(50, 255,
                                                                    50));

} // namespace stag
} // namespace tl
