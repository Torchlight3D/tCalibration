#include "TagDetector.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>

#include "Edge.h"
#include "GrayModel.h"
#include "GLine2D.h"
#include "GLineSegment2D.h"
#include "Gridder.h"
#include "Homography33.h"
#include "MathUtil.h"
#include "Quad.h"
#include "Segment.h"
#include "TagFamily.h"
#include "UnionFindSimple.h"
#include "XYWeight.h"

namespace AprilTags {

using Eigen::Matrix3d;

std::vector<TagDetection> TagDetector::extractTags(const cv::Mat &image)
{
    int width = image.cols;
    int height = image.rows;

    // std::pair<int, int> opticalCenter(width / 2, height / 2);

    // Normalize image
    cv::Mat fimOrigCV;
    image.convertTo(fimOrigCV, CV_32FC1, 1.0 / 255.0);

    cv::Point2f opticalCenter(width / 2, height / 2);

    //================================================================
    // Step one: preprocess image (convert to grayscale) and low pass if
    // necessary

    float sigma = 0;
    float segSigma = 0.8f;

    cv::Mat fimCV, fimSegCV;
    if (sigma > 0) {
        int filtsz = ((int)std::max(3.0f, 3 * sigma)) | 1; // Forces to be odd
        cv::Size ksize(filtsz, filtsz);
        cv::GaussianBlur(fimOrigCV, fimCV, ksize, sigma, sigma);
    }
    else {
        fimOrigCV.copyTo(fimCV);
    }

    if (segSigma > 0) {
        if (segSigma == sigma) {
            fimCV.copyTo(fimSegCV);
        }
        else {
            // blur anew
            int filtsz = ((int)std::max(3.0f, 3 * segSigma)) | 1;
            cv::Size ksize(filtsz, filtsz);
            cv::GaussianBlur(fimOrigCV, fimSegCV, ksize, segSigma, segSigma);
        }
    }
    else {
        fimOrigCV.copyTo(fimSegCV);
    }

    cv::Mat filteredTheta(fimSegCV.size(), CV_32FC1);
    cv::Mat filteredMag(fimSegCV.size(), CV_32FC1);

    filteredMag.forEach<float>([&](float &mag, const int position[]) {
        const int j = position[1];
        const int i = position[0];

        if (i > 0 && j > 0 && i < fimSegCV.rows - 1 && j < fimSegCV.cols - 1) {
            const float Ix =
                fimSegCV.at<float>(i, j + 1) - fimSegCV.at<float>(i, j - 1);
            const float Iy =
                fimSegCV.at<float>(i + 1, j) - fimSegCV.at<float>(i - 1, j);
            mag = Ix * Ix + Iy * Iy;
            filteredTheta.at<float>(i, j) = std::atan2(Iy, Ix);
        }
    });

    //================================================================
    // Step three. Extract edges by grouping pixels with similar
    // thetas together. This is a greedy algorithm: we start with
    // the most similar pixels.  We use 4-connectivity.
    UnionFindSimple uf(fimSegCV.cols * fimSegCV.rows);

    std::vector<Edge> edges;
    edges.reserve(width * height * 4);

    // Bounds on the thetas assigned to this group. Note that because
    // theta is periodic, these are defined such that the average
    // value is contained *within* the interval.
    { // limit scope of storage
        /* Previously all this was on the stack, but this is 1.2MB for 320x240
         * images That's already a problem for OS X (default 512KB thread stack
         * size), could be a problem elsewhere for bigger images... so store on
         * heap */
        std::vector<float>
            storage; // do all the memory in one big block, exception safe
        storage.reserve(width * height * 4);
        float *tmin = &storage[width * height * 0];
        float *tmax = &storage[width * height * 1];
        float *mmin = &storage[width * height * 2];
        float *mmax = &storage[width * height * 3];

        for (int y = 0; y + 1 < height; y++) {
            for (int x = 0; x + 1 < width; x++) {
                float mag0 = filteredMag.at<float>(y, x);
                if (mag0 < Edge::minMag)
                    continue;
                mmax[y * width + x] = mag0;
                mmin[y * width + x] = mag0;

                float theta0 = filteredTheta.at<float>(y, x);
                tmin[y * width + x] = theta0;
                tmax[y * width + x] = theta0;

                // Calculates then adds edges to 'vector<Edge> edges'
                Edge::calcEdges(theta0, x, y, filteredTheta, filteredMag,
                                edges);

                // TODO Would 8 connectivity help for rotated tags?
                // Probably not much, so long as input filtering hasn't been
                // disabled.
            }
        }

        std::stable_sort(edges.begin(), edges.end());
        Edge::mergeEdges(edges, uf, tmin, tmax, mmin, mmax);
    }

    //================================================================
    // Step four: Loop over the pixels again, collecting statistics for each
    // cluster. We will soon fit lines (segments) to these points.

    std::map<int, std::vector<XYWeight>> clusters;
    for (int y = 0; y + 1 < fimSegCV.rows; y++) {
        for (int x = 0; x + 1 < fimSegCV.cols; x++) {
            if (uf.getSetSize(y * fimSegCV.cols + x) <
                Segment::minimumSegmentSize) {
                continue;
            }

            int rep = (int)uf.getRepresentative(y * fimSegCV.cols + x);

            auto it = clusters.find(rep);
            if (it == clusters.end()) {
                clusters[rep] = std::vector<XYWeight>();
                it = clusters.find(rep);
            }
            std::vector<XYWeight> &points = it->second;
            points.push_back(XYWeight{.pos = cv::Point2f(x, y),
                                      .weight = filteredMag.at<float>(x, y)});
        }
    }

    //================================================================
    // Step five: Loop over the clusters, fitting lines (which we call
    // Segments).
    std::vector<Segment> segments; // used in Step six
    for (const auto &[_, points] : clusters) {
        GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);

        // filter short lines
        float length = cv::norm(gseg.p0 - gseg.p1);
        if (length < Segment::minimumLineLength) {
            continue;
        }

        float dy = gseg.p1.y - gseg.p0.y;
        float dx = gseg.p1.x - gseg.p0.x;
        float tmpTheta = std::atan2(dy, dx);

        Segment seg;
        seg.setTheta(tmpTheta);
        seg.setLength(length);

        // We add an extra semantic to segments: the vector
        // p1->p2 will have dark on the left, white on the right.
        // To do this, we'll look at every gradient and each one
        // will vote for which way they think the gradient should
        // go. This is way more retentive than necessary: we
        // could probably sample just one point!

        float flip = 0, noflip = 0;
        for (const auto &xyw : points) {
            float theta = filteredTheta.at<float>(xyw.pos);
            float mag = filteredMag.at<float>(xyw.pos);

            // err *should* be +M_PI/2 for the correct winding, but if we
            // got the wrong winding, it'll be around -M_PI/2.
            float err = MathUtil::mod2pi(theta - seg.getTheta());

            if (err < 0)
                noflip += mag;
            else
                flip += mag;
        }

        if (flip > noflip) {
            seg.setTheta(seg.getTheta() + (float)M_PI);
        }

        float dot =
            dx * std::cos(seg.getTheta()) + dy * std::sin(seg.getTheta());
        if (dot > 0) {
            seg.rP0() = gseg.p1;
            seg.rP1() = gseg.p0;
        }
        else {
            seg.rP0() = gseg.p0;
            seg.rP1() = gseg.p1;
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
        gridder.add(segment.P0(), &segment);
    }

    // Now, find child segments that begin where each parent segment ends.
    for (auto &parentseg : segments) {
        // compute length of the line segment
        const GLine2D parentLine{parentseg.P0(), parentseg.P1()};

        Gridder<Segment>::iterator iter =
            gridder.find(parentseg.P1(), 0.5f * parentseg.getLength());
        while (iter.hasNext()) {
            Segment &child = iter.next();
            if (MathUtil::mod2pi(child.getTheta() - parentseg.getTheta()) > 0) {
                continue;
            }

            // compute intersection of points
            const GLine2D childLine{child.P0(), child.P1()};
            const auto p = parentLine.intersectionWith(childLine);
            if (p.x == -1) {
                continue;
            }

            float parentDist = cv::norm(p - parentseg.P1());
            float childDist = cv::norm(p - child.P0());

            if (std::max(parentDist, childDist) > parentseg.getLength()) {
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
    for (unsigned int i = 0; i < segments.size(); i++) {
        tmp[0] = &segments[i];
        Quad::search(tmp, segments[i], 0, quads, opticalCenter);
    }

    //================================================================
    // Step eight. Decode the quads. For each quad, we first estimate a
    // threshold color to decide between 0 and 1. Then, we read off the
    // bits and see if they make sense.

    std::vector<TagDetection> detections;
    for (const auto &quad : quads) {
        // Find a threshold
        GrayModel blackModel, whiteModel;
        const int dd = 2 * thisTagFamily.blackBorder + thisTagFamily.dimension;

        for (int iy = -1; iy <= dd; iy++) {
            float y = (iy + 0.5f) / dd;
            for (int ix = -1; ix <= dd; ix++) {
                float x = (ix + 0.5f) / dd;
                const auto pxy = quad.interpolate01(x, y);
                int irx = (int)(pxy.x + 0.5);
                int iry = (int)(pxy.y + 0.5);
                if (irx < 0 || irx >= width || iry < 0 || iry >= height)
                    continue;
                float v = fimCV.at<float>(irx, iry);
                if (iy == -1 || iy == dd || ix == -1 || ix == dd)
                    whiteModel.addObservation(x, y, v);
                else if (iy == 0 || iy == (dd - 1) || ix == 0 || ix == (dd - 1))
                    blackModel.addObservation(x, y, v);
            }
        }

        bool bad = false;
        unsigned long long tagCode = 0;
        for (int iy = thisTagFamily.dimension - 1; iy >= 0; iy--) {
            float y = (thisTagFamily.blackBorder + iy + 0.5f) / dd;
            for (int ix = 0; ix < thisTagFamily.dimension; ix++) {
                float x = (thisTagFamily.blackBorder + ix + 0.5f) / dd;
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
                float v = fimCV.at<float>(irx, iry);
                tagCode = tagCode << 1;
                if (v > threshold)
                    tagCode |= 1;
            }
        }

        if (bad) {
            continue;
        }

        TagDetection thisTagDetection;
        thisTagFamily.decode(thisTagDetection, tagCode);

        // compute the homography (and rotate it appropriately)
        auto hom = quad.homography;
        hom.compute();
        thisTagDetection.homography = hom.getH();
        thisTagDetection.hxy = hom.getCXY();

        float c = std::cos(thisTagDetection.rotation * (float)M_PI / 2);
        float s = std::sin(thisTagDetection.rotation * (float)M_PI / 2);
        Matrix3d R;
        R.setZero();
        R(0, 0) = R(1, 1) = c;
        R(0, 1) = -s;
        R(1, 0) = s;
        R(2, 2) = 1;
        Matrix3d tmp;
        tmp = thisTagDetection.homography * R;
        thisTagDetection.homography = tmp;

        // Rotate points in detection according to decoded
        // orientation.  Thus the order of the points in the
        // detection object can be used to determine the
        // orientation of the target.
        const auto bottomLeft = thisTagDetection.interpolate(-1, -1);
        int bestRot = -1;
        float bestDist = FLT_MAX;
        for (int i = 0; i < 4; i++) {
            if (const float dist = cv::norm(bottomLeft - quad.quadPoints[i]);
                dist < bestDist) {
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

    //================================================================
    // Step nine: Some quads may be detected more than once, due to
    // partial occlusion and our aggressive attempts to recover from
    // broken lines. When two quads (with the same id) overlap, we will
    // keep the one with the lowest error, and if the error is the same,
    // the one with the greatest observed perimeter.
    // NOTE: allow multiple non-overlapping detections of the same target.

    std::vector<TagDetection> goodDetections;
    for (const auto &thisTagDetection : detections) {
        bool newFeature = true;
        for (size_t odidx = 0; odidx < goodDetections.size(); odidx++) {
            TagDetection &otherTagDetection = goodDetections[odidx];

            if (thisTagDetection.id != otherTagDetection.id ||
                !thisTagDetection.overlapsTooMuch(otherTagDetection)) {
                continue;
            }

            // There's a conflict.  We must pick one to keep.
            newFeature = false;

            // This detection is worse than the previous one... just don't use
            // it.
            if (thisTagDetection.hammingDistance >
                otherTagDetection.hammingDistance) {
                continue;
            }

            // Otherwise, keep the new one if it either has strictly *lower*
            // error, or greater perimeter.
            if (thisTagDetection.hammingDistance <
                    otherTagDetection.hammingDistance ||
                thisTagDetection.observedPerimeter >
                    otherTagDetection.observedPerimeter) {
                goodDetections[odidx] = thisTagDetection;
            }
        }

        if (newFeature) {
            goodDetections.push_back(thisTagDetection);
        }
    }

    // cout << "AprilTags: edges=" << nEdges << " clusters=" << clusters.size()
    // << " segments=" << segments.size()
    //      << " quads=" << quads.size() << " detections=" << detections.size()
    //      << " unique tags=" << goodDetections.size() << endl;

    return goodDetections;
}

} // namespace AprilTags
