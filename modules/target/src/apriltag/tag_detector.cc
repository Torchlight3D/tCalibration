#include "tag_detector.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/core/mat.hpp>

#include <AxMath/MathBase>

#include "apriltag_types.h"
#include "float_image.h"
#include "gaussian.h"
#include "gridder.h"
#include "gray_model.h"
#include "line_2d.h"
#include "line_segment_2d.h"
#include "quad.h"
#include "union_find_simple.h"

using namespace thoht;

namespace apriltags {

namespace {

cv::Mat toCvMat(const FloatImage &src)
{
    const int rows = src.height();
    const int cols = src.width();
    cv::Mat out{rows, cols, CV_8UC3};
    for (int r{0}; r < rows; ++r) {
        for (int c{0}; c < cols; ++c) {
            const auto val = static_cast<int>(src.at(c, r) * 255.f);
            if ((val & 0xffff00) != 0) {
                LOG(INFO) << "problem... %i" << val;
            }

            cv::Vec3b vec;
            for (int k = 0; k < 3; k++) {
                vec(k) = val;
            }
            out.at<cv::Vec3b>(r, c) = vec;
        }
    }

    return out;
}

FloatImage fromCvMat(const cv::Mat &src)
{
    FloatImage out{src.cols, src.rows};
    for (int r{0}; r < src.rows; ++r) {
        for (int c{0}; c < src.cols; ++c) {
            const auto &vec = src.at<cv::Vec3b>(r, c);
            out.set(c, r, vec(0) / 255.f);
        }
    }

    return out;
}

} // namespace

namespace {

// Minimum intensity gradient for an edge to be recognized
constexpr float kMinEdgeMag{0.004f};

// 30 degrees = maximum acceptable, difference in local orientations
constexpr float kMaxEdgeCost{30.f * math::pi<float> / 180.f};

// was 10000
constexpr int kWidghtScale{100};

// Theta threshold for merging edges
constexpr float kThetaThres{100.f};

// Magnitude threshold for merging edges
constexpr float kMagThres{1200.f};

//! Represents an edge between adjacent pixels in the image.
/*! The edge is encoded by the indices of the two pixels. Edge cost
 *  is proportional to the difference in local orientations.
 */
class Edge
{
public:
    int pixelIdxA;
    int pixelIdxB;
    int cost;

    Edge() : pixelIdxA(0), pixelIdxB(0), cost(0) {}

    inline bool operator<(const Edge &other) const
    {
        return (cost < other.cost);
    }
};

//! Cost of an edge between two adjacent pixels; -1 if no edge here
/*! An edge exists between adjacent pixels if the magnitude of the
  intensity gradient at both pixels is above threshold.  The edge
  cost is proportional to the difference in the local orientation at
  the two pixels.  Lower cost is better.  A cost of -1 means there
  is no edge here (intensity gradien fell below threshold).
 */
int edgeCost(float theta0, float theta1, float mag1)
{
    // mag0 was checked by the main routine so no need
    // to recheck here
    if (mag1 < kMinEdgeMag) {
        return -1;
    }

    const float thetaErr = std::abs(math::mod2pi(theta1 - theta0));
    if (thetaErr > kMaxEdgeCost) {
        return -1;
    }

    const float normErr = thetaErr / kMaxEdgeCost;
    return static_cast<int>(normErr * kWidghtScale);
}

//! Calculates and inserts up to four edges into 'edges', a vector of Edges.
void calcEdges(float theta0, int x, int y, const FloatImage &theta,
               const FloatImage &mag, std::vector<Edge> &edges, size_t &nEdges)
{
    int width = theta.width();
    int thisPixel = y * width + x;

    // horizontal edge
    int cost1 = edgeCost(theta0, theta.at(x + 1, y), mag.at(x + 1, y));
    if (cost1 >= 0) {
        edges[nEdges].cost = cost1;
        edges[nEdges].pixelIdxA = thisPixel;
        edges[nEdges].pixelIdxB = y * width + x + 1;
        ++nEdges;
    }

    // vertical edge
    int cost2 = edgeCost(theta0, theta.at(x, y + 1), mag.at(x, y + 1));
    if (cost2 >= 0) {
        edges[nEdges].cost = cost2;
        edges[nEdges].pixelIdxA = thisPixel;
        edges[nEdges].pixelIdxB = (y + 1) * width + x;
        ++nEdges;
    }

    // downward diagonal edge
    int cost3 = edgeCost(theta0, theta.at(x + 1, y + 1), mag.at(x + 1, y + 1));
    if (cost3 >= 0) {
        edges[nEdges].cost = cost3;
        edges[nEdges].pixelIdxA = thisPixel;
        edges[nEdges].pixelIdxB = (y + 1) * width + x + 1;
        ++nEdges;
    }

    // updward diagonal edge
    int cost4 = (x == 0) ? -1
                         : edgeCost(theta0, theta.at(x - 1, y + 1),
                                    mag.at(x - 1, y + 1));
    if (cost4 >= 0) {
        edges[nEdges].cost = cost4;
        edges[nEdges].pixelIdxA = thisPixel;
        edges[nEdges].pixelIdxB = (y + 1) * width + x - 1;
        ++nEdges;
    }
}

//! Process edges in order of increasing cost, merging clusters if we can do so
//! without exceeding the thetaThresh.
void mergeEdges(std::vector<Edge> &edges, UnionFindSimple &uf, float tmin[],
                float tmax[], float mmin[], float mmax[])
{
    for (const auto &edge : edges) {
        int ida = edge.pixelIdxA;
        int idb = edge.pixelIdxB;

        ida = uf.getRepresentative(ida);
        idb = uf.getRepresentative(idb);

        if (ida == idb)
            continue;

        int sza = uf.getSetSize(ida);
        int szb = uf.getSetSize(idb);

        float tmina = tmin[ida], tmaxa = tmax[ida];
        float tminb = tmin[idb], tmaxb = tmax[idb];

        float costa = (tmaxa - tmina);
        float costb = (tmaxb - tminb);

        // bshift will be a multiple of 2pi that aligns the spans of 'b' with
        // 'a' so that we can properly take the union of them.
        float bshift = math::mod2pi((tmina + tmaxa) / 2, (tminb + tmaxb) / 2) -
                       (tminb + tmaxb) / 2;

        float tminab = std::min(tmina, tminb + bshift);
        float tmaxab = std::max(tmaxa, tmaxb + bshift);

        // corner case that's probably not too useful to handle correctly, oh
        // well.
        if (tmaxab - tminab > math::two_pi<float>) {
            tmaxab = tminab + math::two_pi<float>;
        }

        float mminab = std::min(mmin[ida], mmin[idb]);
        float mmaxab = std::max(mmax[ida], mmax[idb]);

        // merge these two clusters?
        float costab = (tmaxab - tminab);
        if (costab <= (std::min(costa, costb) + kThetaThres / (sza + szb)) &&
            (mmaxab - mminab) <=
                std::min(mmax[ida] - mmin[ida], mmax[idb] - mmin[idb]) +
                    kMagThres / (sza + szb)) {
            int idab = uf.connectNodes(ida, idb);

            tmin[idab] = tminab;
            tmax[idab] = tmaxab;

            mmin[idab] = mminab;
            mmax[idab] = mmaxab;
        }
    }
}

} // namespace

TagDetector::TagDetector(const TagCodes &tagCodes, size_t blackBorder)
    : _tagFamily(tagCodes, blackBorder)
{
}

std::vector<TagDetection> TagDetector::extractTags(const cv::Mat &image)
{
    // TODO:
    // 1. Change to cv::Mat;
    // 2. Check dtype
    int width = image.cols;
    int height = image.rows;
    FloatImage imgSrc{width, height};

    for (int y{0}, i{0}; y < height; y++) {
        for (int x{0}; x < width; x++, i++) {
            imgSrc.set(x, y, image.data[i] / 255.f);
        }
    }
    cv::Point2f opticalCenter{width / 2.f, height / 2.f};

    // 1. Preprocess image (convert to grayscale) and low pass if necessary

    // Gaussian smoothing kernel applied to image (0 == no filter). Used when
    // sampling bits. Filtering is a good idea in cases where A) a cheap camera
    // is introducing artifical sharpening, B) the bayer pattern is creating
    // artifcats, C) the sensor is very noisy and/or has hot/cold pixels.
    // However, filtering makes it harder to decode very small tags. Reasonable
    // values are 0, or [0.8, 1.5].

    constexpr float kSigma{0.f};

    FloatImage imgFiltered = imgSrc;
    if (kSigma > 0.f) {
        const int filterSize = ((int)std::max(3.f, 3 * kSigma)) | 1;
        const std::vector<float> filter =
            Gaussian::makeGaussianFilter(kSigma, filterSize);
        imgFiltered.filterFactoredCentered(filter, filter);
    }

    // 2. Compute the local gradient. We store the direction and
    // magnitude. This step is quite sensitve to noise, since a few bad theta
    // estimates will break up segments, causing us to miss Quads. It is useful
    // to do a Gaussian low pass on this step even if we don't want it for
    // encoding.

    //! Gaussian smoothing kernel applied to image (0 == no filter).
    /*! Used when detecting the outline of the box. It is almost always
     * useful to have some filtering, since the loss of small details
     * won't hurt. Recommended value = 0.8. The case where sigma ==
     * segsigma has been optimized to avoid a redundant filter
     * operation.
     */
    constexpr float kSegmentSigma{0.8f};

    FloatImage imgSegment;
    if (kSegmentSigma > 0.f) {
        if (kSegmentSigma == kSigma) {
            imgSegment = imgFiltered;
        }
        else {
            // blur anew
            int filtsz = ((int)std::max(3.0f, 3 * kSegmentSigma)) | 1;
            std::vector<float> filt =
                Gaussian::makeGaussianFilter(kSegmentSigma, filtsz);
            imgSegment = imgSrc;
            imgSegment.filterFactoredCentered(filt, filt);
        }
    }
    else {
        imgSegment = imgSrc;
    }

    FloatImage imgTheta{imgSegment.width(), imgSegment.height()};
    FloatImage imgMag{imgSegment.width(), imgSegment.height()};

#pragma omp parallel for
    for (int y{1}; y < imgSegment.height() - 1; y++) {
        for (int x{1}; x < imgSegment.width() - 1; x++) {
            float Ix = imgSegment.at(x + 1, y) - imgSegment.at(x - 1, y);
            float Iy = imgSegment.at(x, y + 1) - imgSegment.at(x, y - 1);

            float mag = Ix * Ix + Iy * Iy;

#if 0 // kaess: fast version, but maybe less accurate?
      float theta = math::fast_atan2(Iy, Ix);
#else
            float theta = atan2(Iy, Ix);
#endif

            imgTheta.set(x, y, theta);
            imgMag.set(x, y, mag);
        }
    }

    // 3. Extract edges by grouping pixels with similar
    // thetas together. This is a greedy algorithm: we start with
    // the most similar pixels.  We use 4-connectivity.
    UnionFindSimple unionFind{imgSegment.pixelCount()};

    std::vector<Edge> edges(width * height * 4);
    size_t nEdges = 0;

    // Bounds on the thetas assigned to this group. Note that because
    // theta is periodic, these are defined such that the average
    // value is contained *within* the interval.
    { // limit scope of storage
        /* Previously all this was on the stack, but this is 1.2MB for 320x240
         * images That's already a problem for OS X (default 512KB thread stack
         * size), could be a problem elsewhere for bigger images... so store on
         * heap */
        std::vector<float> storage(width * height * 4);
        float *thetaMin = &storage[width * height * 0];
        float *thetaMax = &storage[width * height * 1];
        float *magMin = &storage[width * height * 2];
        float *magMax = &storage[width * height * 3];

        for (int y{0}; y + 1 < height; y++) {
            for (int x{0}; x + 1 < width; x++) {
                const auto &mag = imgMag.at(x, y);
                if (mag < kMinEdgeMag) {
                    continue;
                }

                magMax[y * width + x] = mag;
                magMin[y * width + x] = mag;

                const auto &theta = imgTheta.at(x, y);
                thetaMin[y * width + x] = theta;
                thetaMax[y * width + x] = theta;

                // Calculates then adds edges to 'vector<Edge> edges'
                calcEdges(theta, x, y, imgTheta, imgMag, edges, nEdges);

                // XXX Would 8 connectivity help for rotated tags?
                // Probably not much, so long as input filtering hasn't been
                // disabled.
            }
        }

        edges.resize(nEdges);
        std::stable_sort(edges.begin(), edges.end());
        mergeEdges(edges, unionFind, thetaMin, thetaMax, magMin, magMax);
    }

    //================================================================
    // Step four: Loop over the pixels again, collecting statistics for each
    // cluster. We will soon fit lines (segments) to these points.

    std::map<int, std::vector<WeightedPointF>> clusters;
    for (int y{0}; y + 1 < imgSegment.height(); y++) {
        for (int x{0}; x + 1 < imgSegment.width(); x++) {
            if (unionFind.getSetSize(y * imgSegment.width() + x) <
                Segment::kMinSegmentSize) {
                continue;
            }

            int rep = unionFind.getRepresentative(y * imgSegment.width() + x);
            auto it = clusters.find(rep);
            if (it == clusters.end()) {
                clusters[rep] = {};
                it = clusters.find(rep);
            }

            auto &points = it->second;
            points.emplace_back(x, y, imgMag.at(x, y));
        }
    }

    // 5. Loop over the clusters, fitting lines (which we call Segments).
    std::vector<Segment> segments;
    for (const auto &[_, weighted_points] : clusters) {
        GLineSegment2D gseg = GLineSegment2D::fitLine(weighted_points);

        // Filter short lines
        float length = cv::norm(gseg.point0() - gseg.point1());
        if (length < Segment::kMinLineLength)
            continue;

        Segment seg;
        float dy = gseg.point1().y - gseg.point0().y;
        float dx = gseg.point1().x - gseg.point0().x;

        float tmpTheta = std::atan2(dy, dx);

        seg.theta = tmpTheta;
        seg.length = length;

        // We add an extra semantic to segments: the vector
        // p1->p2 will have dark on the left, white on the right.
        // To do this, we'll look at every gradient and each one
        // will vote for which way they think the gradient should
        // go. This is way more retentive than necessary: we
        // could probably sample just one point!

        float flip{0.f}, noflip{0.f};
        for (const auto &wpt : weighted_points) {
            float theta = imgTheta.at((int)wpt.point.x, (int)wpt.point.y);
            float mag = imgMag.at((int)wpt.point.x, (int)wpt.point.y);

            // err *should* be +M_PI/2 for the correct winding, but if we
            // got the wrong winding, it'll be around -M_PI/2.
            float err = math::mod2pi(theta - seg.theta);

            if (err < 0)
                noflip += mag;
            else
                flip += mag;
        }

        if (flip > noflip) {
            seg.theta = seg.theta + math::pi<float>;
        }

        float dot = dx * std::cos(seg.theta) + dy * std::sin(seg.theta);
        if (dot > 0) {
            seg.point0 = gseg.point1();
            seg.point1 = gseg.point0();
        }
        else {
            seg.point0 = gseg.point0();
            seg.point1 = gseg.point1();
        }

        segments.push_back(seg);
    }

    // 6. For each segment, find segments that begin where this segment ends.
    // (We will chain segments together next...) The gridder accelerates the
    // search by building (essentially) a 2D hash table.
    Gridder<Segment> gridder(0, 0, width, height, 10);

    // add every segment to the hash table according to the position of the
    // segment's first point. Remember that the first point has a specific
    // meaning due to our left-hand rule above.
    for (auto &segment : segments) {
        gridder.add(segment.point0.x, segment.point0.y, &segment);
    }

    // Now, find child segments that begin where each parent segment ends.
    for (auto &parentseg : segments) {
        // compute length of the line segment
        GLine2D parentLine{parentseg.point0, parentseg.point1};

        auto found = gridder.find(parentseg.point1.x, parentseg.point1.y,
                                  0.5f * parentseg.length);
        while (found.hasNext()) {
            Segment &child = found.next();
            if (math::mod2pi(child.theta - parentseg.theta) > 0) {
                continue;
            }

            // compute intersection of points
            GLine2D childLine{child.point0, child.point1};

            const auto intersection = parentLine.intersectWith(childLine);
            if (intersection.x == -1) {
                continue;
            }

            float parentDist = cv::norm(intersection - parentseg.point1);
            float childDist = cv::norm(intersection - child.point0);

            if (std::max(parentDist, childDist) > parentseg.length) {
                // cout << "intersection too far" << endl;
                continue;
            }

            // everything's OK, this child is a reasonable successor.
            parentseg.children.push_back(&child);
        }
    }

    // 7. Search all connected segments to see if any form a loop of length 4.
    // Add those to the quads list.
    std::vector<Quad> quads;
    std::vector<Segment *> tmp(5);
    for (auto &segment : segments) {
        tmp[0] = &segment;
        Quad::search(imgSrc, tmp, segment, 0, quads, opticalCenter);
    }

    // 8. Decode the quads. For each quad, we first estimate a
    // threshold color to decide between 0 and 1. Then, we read off the
    // bits and see if they make sense.

    std::vector<TagDetection> detections;
    for (auto &quad : quads) {
        // Find a threshold
        GrayModel blackModel, whiteModel;
        const int dd = 2 * _tagFamily.blackBorder + _tagFamily.dimension;
        for (int iy = -1; iy <= dd; iy++) {
            float y = (iy + 0.5f) / dd;
            for (int ix = -1; ix <= dd; ix++) {
                float x = (ix + 0.5f) / dd;
                const auto pxy = quad.interpolate01(x, y);
                int irx = (int)(pxy.x + 0.5);
                int iry = (int)(pxy.y + 0.5);
                if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
                    continue;
                }

                float v = imgFiltered.at(irx, iry);
                if (iy == -1 || iy == dd || ix == -1 || ix == dd)
                    whiteModel.addObservation(x, y, v);
                else if (iy == 0 || iy == (dd - 1) || ix == 0 || ix == (dd - 1))
                    blackModel.addObservation(x, y, v);
            }
        }

        bool bad{false};
        size_t tagCode = 0;
        for (int iy = _tagFamily.dimension - 1; iy >= 0; iy--) {
            float y = (_tagFamily.blackBorder + iy + 0.5f) / dd;
            for (int ix = 0; ix < _tagFamily.dimension; ix++) {
                float x = (_tagFamily.blackBorder + ix + 0.5f) / dd;
                const auto pxy = quad.interpolate01(x, y);
                int irx = (int)(pxy.x + 0.5);
                int iry = (int)(pxy.y + 0.5);
                if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
                    // cout << "*** bad:  irx=" << irx << "  iry=" << iry <<
                    // endl;
                    bad = true;
                    continue;
                }

                float threshold = (blackModel.interpolate(x, y) +
                                   whiteModel.interpolate(x, y)) *
                                  0.5f;
                float v = imgFiltered.at(irx, iry);
                tagCode = tagCode << 1;
                if (v > threshold) {
                    tagCode |= 1;
                }
            }
        }

        if (!bad) {
            TagDetection thisTagDetection;
            _tagFamily.decode(thisTagDetection, tagCode);

            // compute the homography (and rotate it appropriately)
            thisTagDetection.homography = quad.homography.getH();
            thisTagDetection.hxy = quad.homography.getCXY();

            float c =
                std::cos(thisTagDetection.rotation * math::half_pi<float>);
            float s =
                std::sin(thisTagDetection.rotation * math::half_pi<float>);
            Eigen::Matrix3d R;
            R.setZero();
            R(0, 0) = R(1, 1) = c;
            R(0, 1) = -s;
            R(1, 0) = s;
            R(2, 2) = 1;
            thisTagDetection.homography = thisTagDetection.homography * R;

            // Rotate points in detection according to decoded
            // orientation.  Thus the order of the points in the
            // detection object can be used to determine the
            // orientation of the target.
            auto bottomLeft = thisTagDetection.interpolate(-1, -1);
            int bestRot = -1;
            float bestDist = FLT_MAX;
            for (int i = 0; i < 4; i++) {
                const float dist = cv::norm(bottomLeft - quad.quadPoints[i]);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestRot = i;
                }
            }

            for (int i = 0; i < 4; i++) {
                thisTagDetection.p[i] = quad.quadPoints[(i + bestRot) % 4];
            }

            if (thisTagDetection.good) {
                thisTagDetection.cxy = quad.interpolate01(0.5f, 0.5f);
                thisTagDetection.observedPerimeter = quad.observedPerimeter;
                detections.push_back(thisTagDetection);
            }
        }
    }

    // 9. Some quads may be detected more than once, due to
    // partial occlusion and our aggressive attempts to recover from
    // broken lines. When two quads (with the same id) overlap, we will
    // keep the one with the lowest error, and if the error is the same,
    // the one with the greatest observed perimeter.

    std::vector<TagDetection> goodDetections;

    // NOTE: allow multiple non-overlapping detections of the same target.

    for (const auto &detection : detections) {
        bool newFeature{true};

        for (auto &goodDetection : goodDetections) {
            if (detection.id != goodDetection.id ||
                !detection.overlapsTooMuch(goodDetection)) {
                continue;
            }

            // There's a conflict.  We must pick one to keep.
            newFeature = false;

            // This detection is worse than the previous one... just don't use
            // it.
            if (detection.hammingDistance > goodDetection.hammingDistance) {
                continue;
            }

            // Otherwise, keep the new one if it either has strictly *lower*
            // error, or greater perimeter.
            if (detection.hammingDistance < goodDetection.hammingDistance ||
                detection.observedPerimeter > goodDetection.observedPerimeter) {
                goodDetection = detection;
            }
        }

        if (newFeature) {
            goodDetections.push_back(detection);
        }
    }

    VLOG(-1) << "AprilTags: edges=" << nEdges << " clusters=" << clusters.size()
             << " segments=" << segments.size() << " quads=" << quads.size()
             << " detections=" << detections.size()
             << " unique tags=" << goodDetections.size();

    return goodDetections;
}

std::vector<TagDetection> TagDetector::extractTagsKalibr(const cv::Mat &image)
{
    // convert to internal AprilTags image (todo: slow, change internally to
    // OpenCV)
    int width = image.cols;
    int height = image.rows;
    FloatImage fimOrig(width, height);
    int i = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            fimOrig.set(x, y, image.data[i] / 255.);
            i++;
        }
    }
    cv::Point2f opticalCenter(width / 2, height / 2);

    //================================================================
    // Step one: preprocess image (convert to grayscale) and low pass if
    // necessary

    FloatImage fim = fimOrig;

    //! Gaussian smoothing kernel applied to image (0 == no filter).
    /*! Used when sampling bits. Filtering is a good idea in cases
     * where A) a cheap camera is introducing artifical sharpening, B)
     * the bayer pattern is creating artifcats, C) the sensor is very
     * noisy and/or has hot/cold pixels. However, filtering makes it
     * harder to decode very small tags. Reasonable values are 0, or
     * [0.8, 1.5].
     */
    float sigma = 0;

    //! Gaussian smoothing kernel applied to image (0 == no filter).
    /*! Used when detecting the outline of the box. It is almost always
     * useful to have some filtering, since the loss of small details
     * won't hurt. Recommended value = 0.8. The case where sigma ==
     * segsigma has been optimized to avoid a redundant filter
     * operation.
     */
    float segSigma = 0.8f;

    if (sigma > 0) {
        int filtsz = ((int)std::max(3.0f, 3 * sigma)) | 1;
        std::vector<float> filt = Gaussian::makeGaussianFilter(sigma, filtsz);
        fim.filterFactoredCentered(filt, filt);
    }

    //================================================================
    // Step two: Compute the local gradient. We store the direction and
    // magnitude. This step is quite sensitve to noise, since a few bad theta
    // estimates will break up segments, causing us to miss Quads. It is useful
    // to do a Gaussian low pass on this step even if we don't want it for
    // encoding.

    FloatImage fimSeg;
    if (segSigma > 0) {
        if (segSigma == sigma) {
            fimSeg = fim;
        }
        else {
            // blur anew
            int filtsz = ((int)std::max(3.0f, 3 * segSigma)) | 1;
            std::vector<float> filt =
                Gaussian::makeGaussianFilter(segSigma, filtsz);
            fimSeg = fimOrig;
            fimSeg.filterFactoredCentered(filt, filt);
        }
    }
    else {
        fimSeg = fimOrig;
    }

    FloatImage fimTheta(fimSeg.width(), fimSeg.height());
    FloatImage fimMag(fimSeg.width(), fimSeg.height());

#pragma omp parallel for
    for (int y = 1; y < fimSeg.height() - 1; y++) {
        for (int x = 1; x < fimSeg.width() - 1; x++) {
            float Ix = fimSeg.at(x + 1, y) - fimSeg.at(x - 1, y);
            float Iy = fimSeg.at(x, y + 1) - fimSeg.at(x, y - 1);

            float mag = Ix * Ix + Iy * Iy;
#if 0 // kaess: fast version, but maybe less accurate?
      float theta = MathUtil::fast_atan2(Iy, Ix);
#else
            float theta = atan2(Iy, Ix);
#endif

            fimTheta.set(x, y, theta);
            fimMag.set(x, y, mag);
        }
    }

    //================================================================
    // Step three. Extract edges by grouping pixels with similar
    // thetas together. This is a greedy algorithm: we start with
    // the most similar pixels.  We use 4-connectivity.
    UnionFindSimple uf(fimSeg.width() * fimSeg.height());

    std::vector<Edge> edges(width * height * 4);
    size_t nEdges = 0;

    // Bounds on the thetas assigned to this group. Note that because
    // theta is periodic, these are defined such that the average
    // value is contained *within* the interval.
    { // limit scope of storage
        /* Previously all this was on the stack, but this is 1.2MB for 320x240
         * images That's already a problem for OS X (default 512KB thread stack
         * size), could be a problem elsewhere for bigger images... so store on
         * heap */
        std::vector<float> storage(
            width * height *
            4); // do all the memory in one big block, exception safe
        float *tmin = &storage[width * height * 0];
        float *tmax = &storage[width * height * 1];
        float *mmin = &storage[width * height * 2];
        float *mmax = &storage[width * height * 3];

        for (int y = 0; y + 1 < height; y++) {
            for (int x = 0; x + 1 < width; x++) {
                float mag0 = fimMag.at(x, y);
                if (mag0 < kMinEdgeMag)
                    continue;
                mmax[y * width + x] = mag0;
                mmin[y * width + x] = mag0;

                float theta0 = fimTheta.at(x, y);
                tmin[y * width + x] = theta0;
                tmax[y * width + x] = theta0;

                // Calculates then adds edges to 'vector<Edge> edges'
                calcEdges(theta0, x, y, fimTheta, fimMag, edges, nEdges);

                // XXX Would 8 connectivity help for rotated tags?
                // Probably not much, so long as input filtering hasn't been
                // disabled.
            }
        }

        edges.resize(nEdges);
        std::stable_sort(edges.begin(), edges.end());
        mergeEdges(edges, uf, tmin, tmax, mmin, mmax);
    }

    //================================================================
    // Step four: Loop over the pixels again, collecting statistics for each
    // cluster. We will soon fit lines (segments) to these points.

    std::map<int, std::vector<WeightedPointF>> clusters;
    for (int y = 0; y + 1 < fimSeg.height(); y++) {
        for (int x = 0; x + 1 < fimSeg.width(); x++) {
            if (uf.getSetSize(y * fimSeg.width() + x) < Segment::kMinLineLength)
                continue;

            int rep = (int)uf.getRepresentative(y * fimSeg.width() + x);

            auto found = clusters.find(rep);
            if (found == clusters.end()) {
                clusters[rep] = std::vector<WeightedPointF>();
                found = clusters.find(rep);
            }
            std::vector<WeightedPointF> &points = found->second;
            points.push_back(WeightedPointF(x, y, fimMag.at(x, y)));
        }
    }

    //================================================================
    // Step five: Loop over the clusters, fitting lines (which we call
    // Segments).
    std::vector<Segment> segments; // used in Step six
    for (const auto &[_, points] : clusters) {
        GLineSegment2D gseg = GLineSegment2D::fitLine(points);

        // filter short lines
        float length = cv::norm(gseg.point0() - gseg.point1());
        if (length < Segment::kMinLineLength)
            continue;

        Segment seg;
        float dy = gseg.point1().y - gseg.point0().y;
        float dx = gseg.point1().x - gseg.point0().x;

        float tmpTheta = std::atan2(dy, dx);

        seg.theta = tmpTheta;
        seg.length = length;

        // We add an extra semantic to segments: the vector
        // p1->p2 will have dark on the left, white on the right.
        // To do this, we'll look at every gradient and each one
        // will vote for which way they think the gradient should
        // go. This is way more retentive than necessary: we
        // could probably sample just one point!

        float flip = 0, noflip = 0;
        for (size_t i = 0; i < points.size(); i++) {
            auto xyw = points[i];

            float theta = fimTheta.at((int)xyw.point.x, (int)xyw.point.y);
            float mag = fimMag.at((int)xyw.point.x, (int)xyw.point.y);

            // err *should* be +M_PI/2 for the correct winding, but if we
            // got the wrong winding, it'll be around -M_PI/2.
            float err = math::mod2pi(theta - seg.theta);

            if (err < 0) {
                noflip += mag;
            }
            else {
                flip += mag;
            }
        }

        if (flip > noflip) {
            seg.theta = seg.theta + math::pi<float>;
        }

        float dot = dx * std::cos(seg.theta) + dy * std::sin(seg.theta);
        if (dot > 0) {
            seg.point0 = gseg.point1();
            seg.point1 = gseg.point0();
        }
        else {
            seg.point0 = gseg.point0();
            seg.point1 = gseg.point1();
        }

        segments.push_back(seg);
    }

    // Step six: For each segment, find segments that begin where this segment
    // ends. (We will chain segments together next...) The gridder accelerates
    // the search by building (essentially) a 2D hash table.
    Gridder<Segment> gridder(0, 0, width, height, 10);

    // add every segment to the hash table according to the position of the
    // segment's first point. Remember that the first point has a specific
    // meaning due to our left-hand rule above.
    for (auto &segment : segments) {
        gridder.add(segment.point0.x, segment.point0.y, &segment);
    }

    // Now, find child segments that begin where each parent segment ends.
    for (auto &parentseg : segments) {
        // compute length of the line segment
        GLine2D parentLine(parentseg.point0, parentseg.point1);

        auto it = gridder.find(parentseg.point1.x, parentseg.point1.y,
                               0.5f * parentseg.length);
        while (it.hasNext()) {
            Segment &child = it.next();
            if (math::mod2pi(child.theta - parentseg.theta) > 0) {
                continue;
            }

            // compute intersection of points
            GLine2D childLine(child.point0, child.point1);

            const auto p = parentLine.intersectWith(childLine);
            if (p.x == -1) {
                continue;
            }

            float parentDist = cv::norm(p - parentseg.point1);
            float childDist = cv::norm(p - child.point0);

            if (std::max(parentDist, childDist) > parentseg.length) {
                // cout << "intersection too far" << endl;
                continue;
            }

            // everything's OK, this child is a reasonable successor.
            parentseg.children.push_back(&child);
        }
    }

    //================================================================
    // Step seven: Search all connected segments to see if any form a loop of
    // length 4. Add those to the quads list.
    std::vector<Quad> quads;

    std::vector<Segment *> tmp(5);
    for (auto &segment : segments) {
        tmp[0] = &segment;
        Quad::search(fimOrig, tmp, segments[i], 0, quads, opticalCenter);
    }

    //================================================================
    // Step eight. Decode the quads. For each quad, we first estimate a
    // threshold color to decide between 0 and 1. Then, we read off the
    // bits and see if they make sense.

    std::vector<TagDetection> detections;

    for (auto &quad : quads) {
        // Find a threshold
        GrayModel blackModel, whiteModel;
        const int dd = 2 * _tagFamily.blackBorder + _tagFamily.dimension;

        for (int iy = -1; iy <= dd; iy++) {
            float y = (iy + 0.5f) / dd;
            for (int ix = -1; ix <= dd; ix++) {
                float x = (ix + 0.5f) / dd;
                auto pxy = quad.interpolate01(x, y);
                int irx = (int)(pxy.x + 0.5);
                int iry = (int)(pxy.y + 0.5);
                if (irx < 0 || irx >= width || iry < 0 || iry >= height)
                    continue;
                float v = fim.at(irx, iry);
                if (iy == -1 || iy == dd || ix == -1 || ix == dd)
                    whiteModel.addObservation(x, y, v);
                else if (iy == 0 || iy == (dd - 1) || ix == 0 || ix == (dd - 1))
                    blackModel.addObservation(x, y, v);
            }
        }

        bool bad = false;
        unsigned long long tagCode = 0;
        for (int iy = _tagFamily.dimension - 1; iy >= 0; iy--) {
            float y = (_tagFamily.blackBorder + iy + 0.5f) / dd;
            for (int ix = 0; ix < _tagFamily.dimension; ix++) {
                float x = (_tagFamily.blackBorder + ix + 0.5f) / dd;
                auto pxy = quad.interpolate01(x, y);
                int irx = (int)(pxy.x + 0.5);
                int iry = (int)(pxy.y + 0.5);
                if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
                    // cout << "*** bad:  irx=" << irx << "  iry=" << iry <<
                    // endl;
                    bad = true;
                    continue;
                }
                float threshold = (blackModel.interpolate(x, y) +
                                   whiteModel.interpolate(x, y)) *
                                  0.5f;
                float v = fim.at(irx, iry);
                tagCode = tagCode << 1;
                if (v > threshold)
                    tagCode |= 1;
            }
        }

        if (!bad) {
            TagDetection thisTagDetection;
            _tagFamily.decode(thisTagDetection, tagCode);

            // compute the homography (and rotate it appropriately)
            thisTagDetection.homography = quad.homography.getH();
            thisTagDetection.hxy = quad.homography.getCXY();

            float c = std::cos(thisTagDetection.rotation * (float)M_PI / 2);
            float s = std::sin(thisTagDetection.rotation * (float)M_PI / 2);
            Eigen::Matrix3d R;
            R.setZero();
            R(0, 0) = R(1, 1) = c;
            R(0, 1) = -s;
            R(1, 0) = s;
            R(2, 2) = 1;
            Eigen::Matrix3d tmp;
            tmp = thisTagDetection.homography * R;
            thisTagDetection.homography = tmp;

            // Rotate points in detection according to decoded
            // orientation.  Thus the order of the points in the
            // detection object can be used to determine the
            // orientation of the target.
            auto bottomLeft = thisTagDetection.interpolate(-1, -1);
            int bestRot = -1;
            float bestDist = FLT_MAX;
            for (int i = 0; i < 4; i++) {
                float const dist = cv::norm(bottomLeft - quad.quadPoints[i]);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestRot = i;
                }
            }

            for (int i = 0; i < 4; i++)
                thisTagDetection.p[i] = quad.quadPoints[(i + bestRot) % 4];

            if (thisTagDetection.good) {
                thisTagDetection.cxy = quad.interpolate01(0.5f, 0.5f);
                thisTagDetection.observedPerimeter = quad.observedPerimeter;
                detections.push_back(thisTagDetection);
            }
        }
    }

    //================================================================
    // Step nine: Some quads may be detected more than once, due to
    // partial occlusion and our aggressive attempts to recover from
    // broken lines. When two quads (with the same id) overlap, we will
    // keep the one with the lowest error, and if the error is the same,
    // the one with the greatest observed perimeter.

    std::vector<TagDetection> goodDetections;

    // NOTE: allow multiple non-overlapping detections of the same target.
    for (const auto &thisTagDetection : detections) {
        bool newFeature = true;
        for (auto &otherTagDetection : goodDetections) {
            if (thisTagDetection.id != otherTagDetection.id ||
                !thisTagDetection.overlapsTooMuch(otherTagDetection)) {
                continue;
            }

            // There's a conflict.  We must pick one to keep.
            newFeature = false;

            // This detection is worse than the previous one... just don't use
            // it.
            if (thisTagDetection.hammingDistance >
                otherTagDetection.hammingDistance)
                continue;

            // Otherwise, keep the new one if it either has strictly *lower*
            // error, or greater perimeter.
            if (thisTagDetection.hammingDistance <
                    otherTagDetection.hammingDistance ||
                thisTagDetection.observedPerimeter >
                    otherTagDetection.observedPerimeter) {
                otherTagDetection = thisTagDetection;
            }
        }

        if (newFeature) {
            goodDetections.push_back(thisTagDetection);
        }
    }

    LOG(INFO) << "AprilTags: edges=" << nEdges
              << " clusters=" << clusters.size()
              << " segments=" << segments.size() << " quads=" << quads.size()
              << " detections=" << detections.size()
              << " unique tags=" << goodDetections.size();

    return goodDetections;
}

} // namespace apriltags
