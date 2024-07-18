#include "Stag.h"
#include "StagDetector.h"

namespace {

void detectMarkers(
    const cv::Mat& image, int libraryHD,
    std::vector<std::vector<cv::Point2f>>& output_corners,
    std::vector<int>& output_ids, int errorCorrection,
    std::vector<std::vector<cv::Point2f>>* output_rejectedImgPoints);

}

namespace stag {

void detectMarkers(const cv::Mat& image, int libraryHD,
                   std::vector<std::vector<cv::Point2f>>& output_corners,
                   std::vector<int>& output_ids)
{
    ::detectMarkers(image, libraryHD, output_corners, output_ids, -1, nullptr);
}

void detectMarkers(const cv::Mat& image, int libraryHD,
                   std::vector<std::vector<cv::Point2f>>& output_corners,
                   std::vector<int>& output_ids, int errorCorrection)
{
    ::detectMarkers(image, libraryHD, output_corners, output_ids,
                    errorCorrection, nullptr);
}

void detectMarkers(
    const cv::Mat& image, int libraryHD,
    std::vector<std::vector<cv::Point2f>>& output_corners,
    std::vector<int>& output_ids,
    std::vector<std::vector<cv::Point2f>>& output_rejectedImgPoints)
{
    ::detectMarkers(image, libraryHD, output_corners, output_ids, -1,
                    &output_rejectedImgPoints);
}

void detectMarkers(
    const cv::Mat& image, int libraryHD,
    std::vector<std::vector<cv::Point2f>>& output_corners,
    std::vector<int>& output_ids, int errorCorrection,
    std::vector<std::vector<cv::Point2f>>& output_rejectedImgPoints)
{
    ::detectMarkers(image, libraryHD, output_corners, output_ids,
                    errorCorrection, &output_rejectedImgPoints);
}

void drawDetectedMarkers(cv::Mat& image,
                         const std::vector<std::vector<cv::Point2f>>& corners,
                         const std::vector<int>& ids,
                         const cv::Scalar& borderColor)
{
    uint numMarkers = corners.size();

    if (!ids.empty() && ids.size() != corners.size()) {
        throw std::invalid_argument(
            "Number of ids not matching number of markers.");
    }
    bool drawIds = !ids.empty() && ids.size() == corners.size();

    for (uint i = 0; i < numMarkers; i++) {
        const std::vector<cv::Point2f>& marker_corners = corners[i];

        // draw white dot in first corner of marker
        cv::circle(image,
                   cv::Point(static_cast<int>(marker_corners[0].x),
                             static_cast<int>(marker_corners[0].y)),
                   6, cv::Scalar(255, 255, 255), -1);

        // draw white border around marker
        for (int j = 0; j < 4; j++)
            cv::line(image,
                     cv::Point(static_cast<int>(marker_corners[j].x),
                               static_cast<int>(marker_corners[j].y)),
                     cv::Point(static_cast<int>(marker_corners[(j + 1) % 4].x),
                               static_cast<int>(marker_corners[(j + 1) % 4].y)),
                     cv::Scalar(255, 255, 255), 3);

        // draw dot in first corner of marker with specified color
        cv::circle(image,
                   cv::Point(static_cast<int>(marker_corners[0].x),
                             static_cast<int>(marker_corners[0].y)),
                   5, borderColor, -1);

        // draw border around marker with specified color
        for (int j = 0; j < 4; j++)
            cv::line(image,
                     cv::Point(static_cast<int>(marker_corners[j].x),
                               static_cast<int>(marker_corners[j].y)),
                     cv::Point(static_cast<int>(marker_corners[(j + 1) % 4].x),
                               static_cast<int>(marker_corners[(j + 1) % 4].y)),
                     borderColor, 2);

        // draw marker id
        if (drawIds) {
            const int& marker_id = ids[i];

            cv::Point text_pos(
                static_cast<int>((marker_corners[0].x + marker_corners[2].x) /
                                 2),
                static_cast<int>((marker_corners[0].y + marker_corners[2].y) /
                                 2));
            cv::putText(image, std::to_string(marker_id), text_pos,
                        cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255),
                        5);
            cv::putText(image, std::to_string(marker_id), text_pos,
                        cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(50, 50, 255), 2);
        }
    }
}

} // namespace stag

namespace {
void detectMarkers(
    const cv::Mat& image, int libraryHD,
    std::vector<std::vector<cv::Point2f>>& output_corners,
    std::vector<int>& output_ids, int errorCorrection,
    std::vector<std::vector<cv::Point2f>>* output_rejectedImgPoints)
{
    cv::Mat grayImage;

    // convert image to grayscale
    if (image.channels() == 1) {
        grayImage = image;
    }
    else if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    }
    else if (image.channels() == 4) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGRA2GRAY);
    }
    else {
        throw std::invalid_argument(
            "Invalid image color space. Supported color spaces are: "
            "[GRAYSCALE, BGR, BGRA].");
    }

    // check libraryHD
    std::set<int> possibleHDs = {11, 13, 15, 17, 19, 21, 23};
    if (possibleHDs.count(libraryHD) == 0) {
        throw std::invalid_argument(
            "Invalid library HD " + std::to_string(libraryHD) +
            ". Possible values are: [11, 13, 15, 17, 19, 21, 23]");
    }

    // if errorCorrection set to -1, take max possible value for libraryHD
    if (errorCorrection == -1) {
        errorCorrection = (libraryHD - 1) / 2;
    }

    // check errorCorrection
    if (errorCorrection > (libraryHD - 1) / 2 || errorCorrection < 0) {
        throw std::invalid_argument(
            "Invalid error correction value " +
            std::to_string(errorCorrection) + " for library HD " +
            std::to_string(libraryHD) +
            ". Error correction needs to be in range 0 <= HD <= (HD-1)/2.");
    }

    output_corners.clear();
    output_ids.clear();
    if (output_rejectedImgPoints != nullptr) {
        output_rejectedImgPoints->clear();
    }

    StagDetector stag_detector(libraryHD, errorCorrection);
    stag_detector.detectMarkers(grayImage);
    const auto& markers = stag_detector.getMarkers();

    for (const auto& marker : markers) {
        std::vector<cv::Point2f> marker_corners;
        std::transform(marker.corners.begin(), marker.corners.end(),
                       std::back_inserter(marker_corners),
                       [](const cv::Point2d& pt_d) {
                           return cv::Point2f(static_cast<float>(pt_d.x),
                                              static_cast<float>(pt_d.y));
                       });

        output_ids.push_back(marker.id);
        output_corners.emplace_back(std::move(marker_corners));
    }

    if (output_rejectedImgPoints == nullptr) {
        return;
    }

    const auto& falseCandidates = stag_detector.getFalseCandidates();

    for (const auto& falseCandidate : falseCandidates) {
        std::vector<cv::Point2f> rejectedImgPoints;
        std::transform(
            falseCandidate.corners.begin(), falseCandidate.corners.end(),
            std::back_inserter(rejectedImgPoints), [](const cv::Point2d& pt_d) {
                return cv::Point2f(static_cast<float>(pt_d.x),
                                   static_cast<float>(pt_d.y));
            });

        output_rejectedImgPoints->emplace_back(std::move(rejectedImgPoints));
    }
}

} // namespace
