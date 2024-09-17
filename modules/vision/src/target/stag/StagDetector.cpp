#include "StagDetector.h"

#include <opencv2/imgproc.hpp>

#include <tCore/Math>

#include "Drawer.h"
#include "PoseRefiner.h"
#include "utility.h"

namespace tl {
namespace stag {

namespace {
cv::Mat createMatFromPolarCoords(double radius, double radians,
                                 double circleRadius)
{
    cv::Mat point(3, 1, CV_64FC1);
    point.at<double>(0) = 0.5 + cos(radians) * radius * (circleRadius / 0.5);
    point.at<double>(1) = 0.5 - sin(radians) * radius * (circleRadius / 0.5);
    point.at<double>(2) = 1;
    return point;
}
} // namespace

StagDetector::StagDetector(int libraryHD, int inErrorCorrection)
{
    errorCorrection = inErrorCorrection;

    quadDetector = QuadDetector();
    fillCodeLocations();
    decoder = Decoder(libraryHD);
}

void StagDetector::detectMarkers(const cv::Mat& inImage)
{
    image = inImage;
    quadDetector.detectQuads(image, &edInterface);

    std::vector<Quad> quads = quadDetector.getQuads();

    for (auto& quad : quads) {
        quad.estimateHomography();
        Codeword c = readCode(quad);
        int shift;
        int id;
        if (decoder.decode(c, errorCorrection, id, shift)) {
            Marker marker(quad, id);
            marker.shiftCorners2(shift);

            // only add marker if similar not already found
            if (!marker.isSimilarIn(markers)) {
                markers.push_back(marker);
            }
        }
        else
            falseCandidates.push_back(quad);
    }

    for (auto& marker : markers) {
        refineMarkerPose(&edInterface, marker);
    }
}

const std::vector<Marker>& StagDetector::getMarkers() const { return markers; }

const std::vector<Quad>& StagDetector::getFalseCandidates() const
{
    return falseCandidates;
}

void StagDetector::logResults(const std::string& path) const
{
    drawEdgeMap(image, edInterface.getEdgeMap());
    drawLines(image, edInterface.getEDLines());
    drawCorners(image, quadDetector.getCornerGroups());
    drawQuads(image, quadDetector.getQuads());
    drawQuads(image, quadDetector.getDistortedQuads());
    drawMarkers(image, markers);
    drawQuads(image, falseCandidates);
    drawEllipses(image, markers);
}

Codeword StagDetector::readCode(const Quad& q)
{
    // take readings from 48 code locations, 12 black border locations, and 12
    // white border locations
    std::vector<unsigned char> samples(72);

    // a better idea may be creating a list of points to be sampled and let the
    // OpenCV's interpolation function handle the sampling
    for (int i = 0; i < 48; i++) {
        cv::Mat projectedPoint = q.H * codeLocs[i];
        samples[i] = readPixelSafeBilinear(
            image,
            cv::Point2d(
                projectedPoint.at<double>(0) / projectedPoint.at<double>(2),
                projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
    }
    for (int i = 0; i < 12; i++) {
        cv::Mat projectedPoint = q.H * blackLocs[i];
        samples[i + 48] = readPixelSafeBilinear(
            image,
            cv::Point2d(
                projectedPoint.at<double>(0) / projectedPoint.at<double>(2),
                projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
    }
    for (int i = 0; i < 12; i++) {
        cv::Mat projectedPoint = q.H * whiteLocs[i];
        samples[i + 60] = readPixelSafeBilinear(
            image,
            cv::Point2d(
                projectedPoint.at<double>(0) / projectedPoint.at<double>(2),
                projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
    }

    // threshold the readings using Otsu's method
    cv::threshold(samples, samples, 0, 255,
                  cv::THRESH_OTSU + cv::THRESH_BINARY_INV);

    // create a codeword using the thresholded readings
    Codeword c;
    for (int i = 0; i < 48; i++) c[i] = samples[i] / 255;

    return c;
}

void StagDetector::fillCodeLocations()
{
    // fill coordinates to be sampled
    codeLocs = std::vector<cv::Mat>(48);

    // code circles are located in a circle with radius outerCircleRadius
    double outerCircleRadius = 0.4;
    double innerCircleRadius = outerCircleRadius * 0.9;

    // each quadrant is rotated by HALF_PI
    // these part is left as is for self-documenting purposes
    for (int i = 0; i < 4; i++) {
        codeLocs[0 + i * 12] = createMatFromPolarCoords(
            0.088363142525988, 0.785398163397448 + i * half_pi,
            innerCircleRadius);

        codeLocs[1 + i * 12] = createMatFromPolarCoords(
            0.206935928182607, 0.459275804122858 + i * half_pi,
            innerCircleRadius);
        codeLocs[2 + i * 12] = createMatFromPolarCoords(
            0.206935928182607, half_pi - 0.459275804122858 + i * half_pi,
            innerCircleRadius);

        codeLocs[3 + i * 12] = createMatFromPolarCoords(
            0.313672146827381, 0.200579720495241 + i * half_pi,
            innerCircleRadius);
        codeLocs[4 + i * 12] = createMatFromPolarCoords(
            0.327493143484516, 0.591687617505840 + i * half_pi,
            innerCircleRadius);
        codeLocs[5 + i * 12] = createMatFromPolarCoords(
            0.327493143484516, half_pi - 0.591687617505840 + i * half_pi,
            innerCircleRadius);
        codeLocs[6 + i * 12] = createMatFromPolarCoords(
            0.313672146827381, half_pi - 0.200579720495241 + i * half_pi,
            innerCircleRadius);

        codeLocs[7 + i * 12] = createMatFromPolarCoords(
            0.437421957035861, 0.145724938287167 + i * half_pi,
            innerCircleRadius);
        codeLocs[8 + i * 12] = createMatFromPolarCoords(
            0.437226762361658, 0.433363129825345 + i * half_pi,
            innerCircleRadius);
        codeLocs[9 + i * 12] = createMatFromPolarCoords(
            0.430628029742607, 0.785398163397448 + i * half_pi,
            innerCircleRadius);
        codeLocs[10 + i * 12] = createMatFromPolarCoords(
            0.437226762361658, half_pi - 0.433363129825345 + i * half_pi,
            innerCircleRadius);
        codeLocs[11 + i * 12] = createMatFromPolarCoords(
            0.437421957035861, half_pi - 0.145724938287167 + i * half_pi,
            innerCircleRadius);
    }

    double borderDist = 0.045;

    blackLocs = std::vector<cv::Mat>(12);
    whiteLocs = std::vector<cv::Mat>(12);

    for (int i = 0; i < 12; i++) {
        blackLocs[i] = cv::Mat(3, 1, CV_64FC1);
    }
    for (int i = 0; i < 12; i++) {
        whiteLocs[i] = cv::Mat(3, 1, CV_64FC1);
    }

    blackLocs[0].at<double>(0) = borderDist;
    blackLocs[0].at<double>(1) = borderDist * 3;
    blackLocs[0].at<double>(2) = 1;

    blackLocs[1].at<double>(0) = borderDist * 2;
    blackLocs[1].at<double>(1) = borderDist * 2;
    blackLocs[1].at<double>(2) = 1;

    blackLocs[2].at<double>(0) = borderDist * 3;
    blackLocs[2].at<double>(1) = borderDist;
    blackLocs[2].at<double>(2) = 1;

    blackLocs[3].at<double>(0) = 1 - 3 * borderDist;
    blackLocs[3].at<double>(1) = borderDist;
    blackLocs[3].at<double>(2) = 1;

    blackLocs[4].at<double>(0) = 1 - 2 * borderDist;
    blackLocs[4].at<double>(1) = borderDist * 2;
    blackLocs[4].at<double>(2) = 1;

    blackLocs[5].at<double>(0) = 1 - borderDist;
    blackLocs[5].at<double>(1) = borderDist * 3;
    blackLocs[5].at<double>(2) = 1;

    blackLocs[6].at<double>(0) = 1 - borderDist;
    blackLocs[6].at<double>(1) = 1 - 3 * borderDist;
    blackLocs[6].at<double>(2) = 1;

    blackLocs[7].at<double>(0) = 1 - 2 * borderDist;
    blackLocs[7].at<double>(1) = 1 - 2 * borderDist;
    blackLocs[7].at<double>(2) = 1;

    blackLocs[8].at<double>(0) = 1 - 3 * borderDist;
    blackLocs[8].at<double>(1) = 1 - borderDist;
    blackLocs[8].at<double>(2) = 1;

    blackLocs[9].at<double>(0) = borderDist * 3;
    blackLocs[9].at<double>(1) = 1 - borderDist;
    blackLocs[9].at<double>(2) = 1;

    blackLocs[10].at<double>(0) = borderDist * 2;
    blackLocs[10].at<double>(1) = 1 - 2 * borderDist;
    blackLocs[10].at<double>(2) = 1;

    blackLocs[11].at<double>(0) = borderDist;
    blackLocs[11].at<double>(1) = 1 - 3 * borderDist;
    blackLocs[11].at<double>(2) = 1;

    whiteLocs[0].at<double>(0) = 0.25;
    whiteLocs[0].at<double>(1) = -borderDist;
    whiteLocs[0].at<double>(2) = 1;

    whiteLocs[1].at<double>(0) = 0.5;
    whiteLocs[1].at<double>(1) = -borderDist;
    whiteLocs[1].at<double>(2) = 1;

    whiteLocs[2].at<double>(0) = 0.75;
    whiteLocs[2].at<double>(1) = -borderDist;
    whiteLocs[2].at<double>(2) = 1;

    whiteLocs[3].at<double>(0) = 1 + borderDist;
    whiteLocs[3].at<double>(1) = 0.25;
    whiteLocs[3].at<double>(2) = 1;

    whiteLocs[4].at<double>(0) = 1 + borderDist;
    whiteLocs[4].at<double>(1) = 0.5;
    whiteLocs[4].at<double>(2) = 1;

    whiteLocs[5].at<double>(0) = 1 + borderDist;
    whiteLocs[5].at<double>(1) = 0.75;
    whiteLocs[5].at<double>(2) = 1;

    whiteLocs[6].at<double>(0) = 0.75;
    whiteLocs[6].at<double>(1) = 1 + borderDist;
    whiteLocs[6].at<double>(2) = 1;

    whiteLocs[7].at<double>(0) = 0.5;
    whiteLocs[7].at<double>(1) = 1 + borderDist;
    whiteLocs[7].at<double>(2) = 1;

    whiteLocs[8].at<double>(0) = 0.25;
    whiteLocs[8].at<double>(1) = 1 + borderDist;
    whiteLocs[8].at<double>(2) = 1;

    whiteLocs[9].at<double>(0) = -borderDist;
    whiteLocs[9].at<double>(1) = 0.75;
    whiteLocs[9].at<double>(2) = 1;

    whiteLocs[10].at<double>(0) = -borderDist;
    whiteLocs[10].at<double>(1) = 0.5;
    whiteLocs[10].at<double>(2) = 1;

    whiteLocs[11].at<double>(0) = -borderDist;
    whiteLocs[11].at<double>(1) = 0.25;
    whiteLocs[11].at<double>(2) = 1;
}

} // namespace stag
} // namespace tl
