#include "cvtools.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace cvtools {

#define sc static_cast

cv::Mat plotScatterXY(const std::vector<cv::Point2d> &P, const cv::Size size,
                      double &minX, double &maxX, double &minY)
{
    cv::Scalar backgroundColor(255, 255, 255);
    cv::Scalar axisColor(0, 0, 255);
    cv::Scalar markerColor(255, 0, 0);

    // automatically determine axis limits
    if (std::abs(maxX - minX) < 1e-10) {
        auto mmit = std::minmax_element(
            P.begin(), P.end(),
            [](cv::Point2d a, cv::Point2d b) { return a.x < b.x; });
        minX = mmit.first->x;
        maxX = mmit.second->x;

        minY = (*std::min_element(
                    P.begin(), P.end(),
                    [](cv::Point2d a, cv::Point2d b) { return a.y < b.y; }))
                   .y;
    }

    cv::Mat plotIm(size, CV_8UC3, backgroundColor);

    // scale [px/unit]
    double scale = size.width / (maxX - minX);

    double maxY = size.height / scale + minY;

    auto coordsToPx = [&](const cv::Point2d &coords) -> cv::Point2d {
        return {(coords.x - minX) * scale,
                size.height + (minY - coords.y) * scale};
    };

    cv::Point2d origin = coordsToPx({0.0, 0.0}); //[px in image coordinates]

    // x-axis
    cv::line(plotIm, cv::Point2d(0, origin.y),
             cv::Point2d(size.width, origin.y), axisColor);
    // y-axis
    cv::line(plotIm, cv::Point2d(origin.x, 0),
             cv::Point2d(origin.x, size.height), axisColor);

    // tick-marks
    double tickHalfLength = (size.width / 100.0) / scale;
    for (auto xc = std::ceil(minX); xc <= std::floor(maxX); xc++) {
        cv::line(plotIm, coordsToPx(cv::Point2d(xc, -tickHalfLength)),
                 coordsToPx(cv::Point2d(xc, tickHalfLength)), axisColor);
    }
    for (auto yc = std::ceil(minY); yc <= std::floor(maxY); yc++) {
        cv::line(plotIm, coordsToPx(cv::Point2d(-tickHalfLength, yc)),
                 coordsToPx(cv::Point2d(tickHalfLength, yc)), axisColor);
    }

    for (const auto &pt : P) {
        cv::drawMarker(plotIm, coordsToPx(pt), markerColor);
    }

    return plotIm;
}

cv::Vec3f applyHomTransform(const cv::Matx44f &T, const cv::Vec3f &p)
{
    cv::Vec4f pHom(p[0], p[1], p[2], 1.0);
    pHom = T * pHom;

    return cv::Vec3f(pHom[0] / pHom[3], pHom[1] / pHom[3], pHom[2] / pHom[3]);
}

cv::Vec2f applyHomTransform(const cv::Matx33f &T, const cv::Vec2f &p)
{
    cv::Vec3f pHom(p[0], p[1], 1.0);
    pHom = T * pHom;

    return cv::Vec2f(pHom[0] / pHom[2], pHom[1] / pHom[2]);
}

std::vector<cv::Point2f> applyHomTransform(const cv::Matx33f &T,
                                           const std::vector<cv::Point2f> &ps)
{
    std::vector<cv::Point2f> psTransformed;
    psTransformed.reserve(ps.size());
    for (auto &p : ps) {
        psTransformed.emplace_back(applyHomTransform(T, p));
    }

    return psTransformed;
}

std::vector<size_t> findNearestNeighborsAngleSorted(
    const cv::KeyPoint queryPoint, const std::vector<cv::KeyPoint> searchPoints,
    int N)
{
    struct neighbour
    {
        size_t idx;
        float distSq;
        float angle;
    };

    // Brute force NN search
    std::vector<neighbour> nn;
    for (auto j = 0u; j < searchPoints.size(); j++) {
        if (queryPoint.pt == searchPoints[j].pt)
            continue;

        float distSq =
            sc<float>(cv::norm(cv::Vec2f(queryPoint.pt),
                               cv::Vec2f(searchPoints[j].pt), cv::NORM_L2SQR));

        neighbour n{j, distSq, 0};

        if (nn.size() < sc<size_t>(N)) {
            nn.push_back(n);
        }
        else {
            if (distSq < nn[sc<size_t>(N - 1)].distSq) {
                nn[sc<size_t>(N - 1)] = n;
            }
        }

        std::sort(nn.begin(), nn.end(),
                  [](neighbour a, neighbour b) { return a.distSq < b.distSq; });
    }

    // Now sort neighbours according to angle
    for (auto &n : nn) {
        n.angle = std::atan2(searchPoints[n.idx].pt.x - queryPoint.pt.x,
                             searchPoints[n.idx].pt.y - queryPoint.pt.y);
    }
    std::sort(nn.begin(), nn.end(),
              [](neighbour a, neighbour b) { return a.angle < b.angle; });

    // Return idx vector
    std::vector<size_t> idxs;
    for (auto n : nn) idxs.push_back(n.idx);

    return idxs;
}

cv::Mat fitHomography(std::vector<cv::Point2f> q1, std::vector<cv::Point2f> q2,
                      float &ssd)
{
    cv::Mat H = cv::findHomography(q1, q2, 0);

    std::vector<cv::Point2f> q1T = applyHomTransform(H, q1);

    ssd = 0;
    for (auto i = 0u; i < q1.size(); i++) {
        ssd += static_cast<float>(
            cv::norm(cv::Vec2f(q1T[i] - q2[i]), cv::NORM_L2SQR));
    }

    return H;
}

bool findPartialCirclesGrid(const cv::Mat &im, std::vector<cv::Point2f> &q,
                            std::vector<cv::Point3f> &Q,
                            const float circleSpacing)
{
    cv::SimpleBlobDetector::Params parameters;
    parameters.blobColor = 255;
    parameters.filterByArea = true;
    parameters.filterByColor = true;
    parameters.filterByCircularity = true;
    parameters.filterByInertia = true;
    parameters.filterByConvexity = true;
    parameters.minArea = 50;
    parameters.maxArea = 1000;
    parameters.minCircularity = 0.85f; // note: very sensitive parameter.
    parameters.maxCircularity = 1.1f;
    parameters.minInertiaRatio = 0.4f;
    parameters.maxInertiaRatio = 1.1f;
    parameters.minConvexity = 0.9f;
    parameters.maxConvexity = 1.1f;

    cv::Ptr<cv::SimpleBlobDetector> blobDetector =
        cv::SimpleBlobDetector::create(parameters);

    std::vector<cv::KeyPoint> keypoints;
    blobDetector->detect(im, keypoints);

    // cv::imwrite("im.png", im);

    if (keypoints.size() < 5)
        return false;

    // remove keypoint too close to image border
    std::vector<cv::KeyPoint> keypointsFiltered;
    for (auto &k : keypoints) {
        const float d = 3; // px
        float r = k.size / 2.0f;
        if (k.pt.x > (r + d) && k.pt.x < (im.cols - (r + d)) &&
            k.pt.y > (r + d) && k.pt.y < (im.rows - (r + d)))
            keypointsFiltered.emplace_back(k);
    }
    keypoints = keypointsFiltered;

    // Move through all detections
    std::vector<std::pair<cv::Point, cv::Point2f>> corrBest;
    for (auto i = 0u; i < keypoints.size(); i++) {
        std::vector<size_t> nn =
            findNearestNeighborsAngleSorted(keypoints[i], keypoints, 4);

        // center, neighborhood
        std::vector<cv::Point2f> qIm{keypoints[i].pt};
        for (auto &n : nn) qIm.push_back(keypoints[n].pt);

        // Assuming the nine points form a "8-neighborhood", we fit a homography
        //    std::vector<cv::Point2f> qObj = {{0, 0}, {-1, 0}, {-1, 1}, {0, 1},
        //    {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
        std::vector<cv::Point2f> qObj = {
            {0, 0}, {-1, 0}, {0, 1}, {1, 0}, {0, -1}};

        float ssd;
        cv::Mat H = fitHomography(qIm, qObj, ssd);

        // If ssd is high, we probably did not get the desired 8-neighborhood
        if (ssd > 0.01f)
            continue;

        // Transform image corners
        std::vector<cv::Point2f> corners{{-0.5f, -0.5f},
                                         {im.cols - 0.5f, 0.5f},
                                         {im.cols - 0.5f, im.rows - 0.5f},
                                         {0.5f, im.rows - 0.5f}};

        std::vector<cv::Point2f> cornersObj =
            cvtools::applyHomTransform(cv::Matx33f(H), corners);

        // Get axis-aligned bounding rect
        cv::Rect r = cv::boundingRect(cornersObj);
        int minX = r.x;
        int minY = r.y;
        int maxX = r.x + r.width;
        int maxY = r.y + r.height;

#if DEBUG_OUTPUT
        cv::Mat imColor;
        cv::cvtColor(im, imColor, cv::COLOR_GRAY2BGR);
        cv::drawKeypoints(im, keypoints, imColor);
#endif

        cv::Mat Hinv = H.inv();

        std::vector<std::pair<cv::Point, cv::Point2f>> idealPoints;
        for (int i = minY; i < maxY; i++) {
            for (int j = minX; j < maxX; j++) {
                cv::Point2f imPoint = cvtools::applyHomTransform(
                    cv::Matx33f(H).inv(), cv::Point2f(j, i));
                idealPoints.push_back(std::make_pair(cv::Point(j, i), imPoint));
#if DEBUG_OUTPUT
                cv::drawMarker(imColor, imPoint, cv::Scalar(255, 0, 0),
                               cv::MARKER_CROSS, 20, 1);
#endif
            }
        }

#if DEBUG_OUTPUT
        cv::drawMarker(imColor, qIm[0], cv::Scalar(0, 255, 0));
        for (auto i = 1u; i < qIm.size(); i++)
            cv::drawMarker(imColor, qIm[i], cv::Scalar(0, 0, 255));

        cv::imwrite("imColor.png", imColor);
#endif

        // Get correspondences
        std::vector<std::pair<cv::Point, cv::Point2f>> corr;
        for (auto ip : idealPoints) {
            for (auto k : keypoints) {
                if (cv::norm(cv::Vec2f(ip.second - k.pt), cv::NORM_L2SQR) <
                    55) { // pxsq

                    corr.push_back(std::make_pair(ip.first, k.pt));
                }
            }
        }

        if (corr.size() > corrBest.size()) {
            corrBest = corr;
            //      iBest = i;
        }
    }

    // Fit homography with all current correspondences
    std::vector<cv::Point> idealPoints(corrBest.size());
    q.resize(corrBest.size());

    for (auto i = 0u; i < corrBest.size(); i++) {
        idealPoints[i] = corrBest[i].first;
        q[i] = corrBest[i].second;
    }

    // Offset Q 'indices' such that all are positive
    cv::Rect r = cv::boundingRect(idealPoints);
    Q.resize(corrBest.size());
    for (auto i = 0u; i < corrBest.size(); i++)
        Q[i] = circleSpacing *
               cv::Point3f(idealPoints[i].x - r.x, idealPoints[i].y - r.y, 0.0);

    if (corrBest.size() > 10)
        return true;
    else
        return false;
}

// Phase correlation image registration including scale, rotation and
// translational shift
void phaseCorrelate(const cv::Mat &im1, const cv::Mat &im2, float &scale,
                    float &angle, cv::Point2f &shift)
{
    assert(im1.size() == im2.size());
    assert(im1.type() == im2.type());

    scale = 1.0;
    angle = 0.0;

    //    cv::Mat im1Float, im2Float;
    //    im1.convertTo(im1Float, CV_32F);
    //    im2.convertTo(im2Float, CV_32F);

    //    cv::Mat im1LogPolar = cvtools::logPolar(im1Float, 100.0);
    //    cv::Mat im2LogPolar = cvtools::logPolar(im2Float, 100.0);

    // hanning window
    cv::Mat window;
    cv::createHanningWindow(window, im1.size(), CV_32F);

    //    // determine scale and rotation
    //    cv::Point2f scaleRotation =
    //    phasecorrelation::phaseCorrelate(im1LogPolar, im2LogPolar, window);

    //    // convert scale to proper scale
    //    scale = cv::exp(scaleRotation.x / 100.0);

    //    // convert rotation angle to degrees
    //    angle = -scaleRotation.y * 180.0/(im1.cols/2.0);

    //    // correct for scale and rotation
    //    cv::Mat im1ScaledRotated;

    //    cv::Mat scaleRotationMatrix =
    //    cv::getRotationMatrix2D(cv::Point2f(im1.cols/2.0, im1.rows/2.0),
    //    angle, scale); cv::warpAffine(im1Float, im1ScaledRotated,
    //    scaleRotationMatrix, im1Float.size());

    // determine translational shift
    // shift = phasecorrelation::phaseCorrelate(im1, im2, window);
}

// Log polar image transformation with log scaling factor (to bring intensities
// into proper range)
cv::Mat logPolar(const cv::Mat &image, float scale)
{
    cv::Mat result(image.size(), image.type());

    cv::logPolar(image, result, cv::Point2f(image.cols / 2.0, image.rows / 2.0),
                 scale, 0);
    return result;
}

// Forward distortion of points. The inverse of the undistortion in
// cv::initUndistortRectifyMap(). Inspired by Pascal Thomet,
// http://code.opencv.org/issues/1387#note-11 Convention for distortion
// parameters:
// http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
void initDistortMap(const cv::Matx33f cameraMatrix,
                    const cv::Vec<float, 5> distCoeffs, const cv::Size size,
                    cv::Mat &map1, cv::Mat &map2)
{
    float fx = cameraMatrix(0, 0);
    float fy = cameraMatrix(1, 1);
    float ux = cameraMatrix(0, 2);
    float uy = cameraMatrix(1, 2);

    float k1 = distCoeffs[0];
    float k2 = distCoeffs[1];
    float p1 = distCoeffs[2];
    float p2 = distCoeffs[3];
    float k3 = distCoeffs[4];

    map1.create(size, CV_32F);
    map2.create(size, CV_32F);

    for (int col = 0; col < size.width; col++) {
        for (int row = 0; row < size.height; row++) {
            // move origo to principal point and convert using focal length
            float x = (col - ux) / fx;
            float y = (row - uy) / fy;

            float xCorrected, yCorrected;

            // Step 1 : correct distortion
            float r2 = x * x + y * y;
            // radial
            xCorrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
            yCorrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
            // tangential
            xCorrected =
                xCorrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
            yCorrected =
                yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);

            // convert back to pixel coordinates
            float col_displaced = xCorrected * fx + ux;
            float row_displaced = yCorrected * fy + uy;

            // correct the vector in the opposite direction
            map1.at<float>(row, col) = col + (col - col_displaced);
            map2.at<float>(row, col) = row + (row - row_displaced);
        }
    }
}

void diamondDownsample(cv::InputArray _src, cv::OutputArray _dst)
{
    _dst.create(_src.rows(), _src.cols() / 2, _src.type());

    // FIXME: How to make this function works for generic types
    const auto src = _src.getMat();
    auto dst = _dst.getMat();
    for (int col = 0; col < src.cols; col++) {
        for (int row = 0; row < src.rows; row++) {
            dst.at<cv::Vec3b>(row, col) =
                src.at<cv::Vec3b>(row, col * 2 + row % 2);
        }
    }
}

cv::Mat histimage(cv::Mat histogram)
{
    cv::Mat histImage(512, 640, CV_8UC3, cv::Scalar(0));

    // Normalize the result to [ 2, histImage.rows-2 ]
    cv::normalize(histogram, histogram, 2, histImage.rows - 2, cv::NORM_MINMAX,
                  -1, cv::Mat());

    float bin_w = (float)histImage.cols / (float)histogram.rows;

    // Draw main histogram
    for (int i = 1; i < histogram.rows - 10; i++) {
        cv::line(
            histImage,
            cv::Point(bin_w * (i - 1),
                      histImage.rows - cvRound(histogram.at<float>(i - 1))),
            cv::Point(bin_w * (i),
                      histImage.rows - cvRound(histogram.at<float>(i))),
            cv::Scalar(255, 255, 255), 2, 4);
    }

    // Draw red max
    for (int i = histogram.rows - 10; i < histogram.rows; i++) {
        cv::line(
            histImage,
            cv::Point(bin_w * (i - 1),
                      histImage.rows - cvRound(histogram.at<float>(i - 1))),
            cv::Point(bin_w * (i),
                      histImage.rows - cvRound(histogram.at<float>(i))),
            cv::Scalar(255, 0, 0), 2, 4);
    }

    return histImage;
}

} // namespace cvtools
