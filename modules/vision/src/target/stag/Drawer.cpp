#include "Drawer.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "colors.h"

namespace tl {
namespace stag {

namespace {
void colorAPixel(cv::Mat& img, int x, int y, cv::Scalar color, int dotWidth)
{
    for (int i = y - dotWidth; i < y + dotWidth + 1; i++) {
        for (int j = x - dotWidth; j < x + dotWidth + 1; j++) {
            if ((i >= 0) && (i < img.rows) && (j >= 0) && (j < img.cols)) {
                img.at<cv::Vec3b>(i, j)[0] = color.val[0];
                img.at<cv::Vec3b>(i, j)[1] = color.val[1];
                img.at<cv::Vec3b>(i, j)[2] = color.val[2];
            }
        }
    }
}
} // namespace

void drawEdgeMap(cv::InputOutputArray image, const EdgeMap* edgeMap)
{
    cv::Mat bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    }
    else {
        bgr = image.getMat();
    }

    int dotWidth = 1;
    int whiteDotWidth = 2;

    for (int i = 0; i < edgeMap->noSegments; i++) {
        for (int j = 0; j < edgeMap->segments[i].noPixels; j++) {
            colorAPixel(bgr, edgeMap->segments[i].pixels[j].c,
                        edgeMap->segments[i].pixels[j].r, CV_RGB(255, 255, 255),
                        whiteDotWidth);
        }
    }

    for (int i = 0; i < edgeMap->noSegments; i++) {
        for (int j = 0; j < edgeMap->segments[i].noPixels; j++) {
            colorAPixel(bgr, edgeMap->segments[i].pixels[j].c,
                        edgeMap->segments[i].pixels[j].r,
                        colors[i % colors.size()], dotWidth);
        }
    }
}

void drawLines(cv::InputOutputArray image, EDLines* edLines)
{
    cv::Mat bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    }
    else {
        bgr = image.getMat();
    }

    int currSegment = -1;
    for (int i = 0; i < edLines->noLines; i++) {
        if (edLines->lines[i].segmentNo != currSegment)
            currSegment = edLines->lines[i].segmentNo;

        cv::line(bgr, cv::Point(edLines->lines[i].sx, edLines->lines[i].sy),
                 cv::Point(edLines->lines[i].ex, edLines->lines[i].ey),
                 CV_RGB(255, 255, 255), 3, cv::LINE_AA);
        cv::line(bgr, cv::Point(edLines->lines[i].sx, edLines->lines[i].sy),
                 cv::Point(edLines->lines[i].ex, edLines->lines[i].ey),
                 colors[i % colors.size()], 2, cv::LINE_AA);
    }
}

void drawCorners(cv::InputOutputArray image,
                 const std::vector<std::vector<Corner>>& cornerGroups)
{
    cv::Mat bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    }
    else {
        bgr = image.getMat();
    }

    for (int i = 0; i < cornerGroups.size(); i++) {
        for (int j = 0; j < cornerGroups[i].size(); j++) {
            cv::circle(bgr, cornerGroups[i][j].loc, 4, CV_RGB(255, 255, 255),
                       -1, cv::LINE_AA);
            cv::circle(bgr, cornerGroups[i][j].loc, 3,
                       colors[i % colors.size()], -1, cv::LINE_AA);
        }
    }
}

void drawQuads(cv::InputOutputArray image, const std::vector<Quad>& quads)
{
    cv::Mat bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    }
    else {
        bgr = image.getMat();
    }

    for (int i = 0; i < quads.size(); i++) {
        const auto& corners = quads[i].corners;

        // cv::circle(bgrMat, corners[0], 6, CV_RGB(255, 255, 255), -1,
        //            cv::LINE_AA);

        for (int j = 0; j < 4; j++) {
            cv::line(bgr, corners[j], corners[(j + 1) % 4],
                     CV_RGB(255, 255, 255), 3, cv::LINE_AA);
        }

        // cv::circle(bgrMat, corners[0], 5, CV_RGB(50, 255, 50), -1,
        // cv::LINE_AA);

        for (int j = 0; j < 4; j++) {
            cv::line(bgr, corners[j], corners[(j + 1) % 4], CV_RGB(50, 255, 50),
                     2, cv::LINE_AA);
        }
    }
}

void drawMarkers(cv::InputOutputArray image, const std::vector<Marker>& markers)
{
    cv::Mat bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    }
    else {
        bgr = image.getMat();
    }

    for (int i = 0; i < markers.size(); i++) {
        const auto& corners = markers[i].corners;
        const auto& center = markers[i].center;

        cv::circle(bgr, corners[0], 6, CV_RGB(255, 255, 255), -1, cv::LINE_AA);
        for (int j = 0; j < 4; j++) {
            cv::line(bgr, corners[j], corners[(j + 1) % 4],
                     CV_RGB(255, 255, 255), 3, cv::LINE_AA);
        }

        cv::circle(bgr, corners[0], 5, CV_RGB(50, 255, 50), -1, cv::LINE_AA);
        for (int j = 0; j < 4; j++) {
            cv::line(bgr, corners[j], corners[(j + 1) % 4], CV_RGB(50, 255, 50),
                     2, cv::LINE_AA);
        }

        cv::circle(bgr, center, 6, CV_RGB(255, 255, 255), -1, cv::LINE_AA);
        cv::circle(bgr, center, 5, CV_RGB(50, 255, 50), -1, cv::LINE_AA);

        cv::putText(bgr, std::to_string(markers[i].id), center,
                    cv::FONT_HERSHEY_DUPLEX, 2, CV_RGB(255, 255, 255), 5,
                    cv::LINE_AA);
        cv::putText(bgr, std::to_string(markers[i].id), center,
                    cv::FONT_HERSHEY_DUPLEX, 2, CV_RGB(255, 50, 50), 2,
                    cv::LINE_AA);
    }
}

void drawEllipses(cv::InputOutputArray image,
                  const std::vector<Marker>& markers)
{
    cv::Mat bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    }
    else {
        bgr = image.getMat();
    }

    int dotWidth = 1;
    int whiteDotWidth = 2;

    for (int i = 0; i < markers.size(); i++) {
        // skip if the ellipse is not localized
        if (markers[i].C.size().width == 1)
            continue;

        for (int y = 0; y < image.size().height; y++) {
            std::vector<double> xOfPointsOnConic;

            double a = markers[i].C.at<double>(0, 0);
            double b = markers[i].C.at<double>(0, 1) * 2 * y +
                       markers[i].C.at<double>(0, 2) * 2;
            double c = markers[i].C.at<double>(1, 1) * y * y +
                       markers[i].C.at<double>(1, 2) * 2 * y +
                       markers[i].C.at<double>(2, 2);

            double disc = b * b - 4 * a * c;

            if (disc == 0)
                xOfPointsOnConic.push_back(-b / (2 * a));
            else if (disc > 0) {
                xOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
                xOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
            }

            for (int j = 0; j < xOfPointsOnConic.size(); j++) {
                xOfPointsOnConic[j] = round(xOfPointsOnConic[j]);

                if ((xOfPointsOnConic[j] < 0) ||
                    (xOfPointsOnConic[j] >= image.size().width))
                    continue;

                colorAPixel(bgr, xOfPointsOnConic[j], y, CV_RGB(255, 255, 255),
                            whiteDotWidth);
            }
        }

        for (int x = 0; x < image.size().width; x++) {
            std::vector<double> yOfPointsOnConic;

            double a = markers[i].C.at<double>(1, 1);
            double b = markers[i].C.at<double>(0, 1) * 2 * x +
                       markers[i].C.at<double>(1, 2) * 2;
            double c = markers[i].C.at<double>(0, 0) * x * x +
                       markers[i].C.at<double>(0, 2) * 2 * x +
                       markers[i].C.at<double>(2, 2);

            double disc = b * b - 4 * a * c;

            if (disc == 0)
                yOfPointsOnConic.push_back(-b / (2 * a));
            else if (disc > 0) {
                yOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
                yOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
            }

            for (int j = 0; j < yOfPointsOnConic.size(); j++) {
                yOfPointsOnConic[j] = round(yOfPointsOnConic[j]);

                if ((yOfPointsOnConic[j] < 0) ||
                    (yOfPointsOnConic[j] >= image.size().height))
                    continue;

                colorAPixel(bgr, x, yOfPointsOnConic[j], CV_RGB(255, 255, 255),
                            whiteDotWidth);
            }
        }

        for (int y = 0; y < image.size().height; y++) {
            std::vector<double> xOfPointsOnConic;

            double a = markers[i].C.at<double>(0, 0);
            double b = markers[i].C.at<double>(0, 1) * 2 * y +
                       markers[i].C.at<double>(0, 2) * 2;
            double c = markers[i].C.at<double>(1, 1) * y * y +
                       markers[i].C.at<double>(1, 2) * 2 * y +
                       markers[i].C.at<double>(2, 2);

            double disc = b * b - 4 * a * c;

            if (disc == 0)
                xOfPointsOnConic.push_back(-b / (2 * a));
            else if (disc > 0) {
                xOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
                xOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
            }

            for (int j = 0; j < xOfPointsOnConic.size(); j++) {
                xOfPointsOnConic[j] = round(xOfPointsOnConic[j]);

                if ((xOfPointsOnConic[j] < 0) ||
                    (xOfPointsOnConic[j] >= image.size().width))
                    continue;

                colorAPixel(bgr, xOfPointsOnConic[j], y, CV_RGB(50, 255, 50),
                            dotWidth);
            }
        }

        for (int x = 0; x < image.size().width; x++) {
            std::vector<double> yOfPointsOnConic;

            double a = markers[i].C.at<double>(1, 1);
            double b = markers[i].C.at<double>(0, 1) * 2 * x +
                       markers[i].C.at<double>(1, 2) * 2;
            double c = markers[i].C.at<double>(0, 0) * x * x +
                       markers[i].C.at<double>(0, 2) * 2 * x +
                       markers[i].C.at<double>(2, 2);

            double disc = b * b - 4 * a * c;

            if (disc == 0)
                yOfPointsOnConic.push_back(-b / (2 * a));
            else if (disc > 0) {
                yOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
                yOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
            }

            for (int j = 0; j < yOfPointsOnConic.size(); j++) {
                yOfPointsOnConic[j] = round(yOfPointsOnConic[j]);

                if ((yOfPointsOnConic[j] < 0) ||
                    (yOfPointsOnConic[j] >= image.size().height))
                    continue;

                colorAPixel(bgr, x, yOfPointsOnConic[j], CV_RGB(50, 255, 50),
                            dotWidth);
            }
        }
    }
}

} // namespace stag
} // namespace tl
