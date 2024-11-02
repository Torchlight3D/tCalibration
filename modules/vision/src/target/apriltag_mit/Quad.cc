#include <Eigen/Dense>

#include "MathUtil.h"
#include "GLine2D.h"
#include "Quad.h"
#include "Segment.h"

namespace AprilTags {

Quad::Quad(const std::vector<cv::Point2f>& p, const cv::Point2f& opticalCenter)
    : quadPoints(p), segments(), observedPerimeter(), homography(opticalCenter)
{
#ifdef STABLE_H
    std::vector<cv::Point2f> srcPts;
    srcPts.emplace_back(-1.f, -1.f);
    srcPts.emplace_back(1.f, -1.f);
    srcPts.emplace_back(1.f, 1.f);
    srcPts.emplace_back(-1.f, 1.f);
    homography.setCorrespondences(srcPts, p);
#else
    homography.addCorrespondence(-1, -1, quadPoints[0].x, quadPoints[0].y);
    homography.addCorrespondence(1, -1, quadPoints[1].x, quadPoints[1].y);
    homography.addCorrespondence(1, 1, quadPoints[2].x, quadPoints[2].y);
    homography.addCorrespondence(-1, 1, quadPoints[3].x, quadPoints[3].y);
#endif

#ifdef INTERPOLATE
    p0 = Eigen::Vector2f(p[0].x, p[0].y);
    p3 = Eigen::Vector2f(p[3].x, p[3].y);
    p01 = (Eigen::Vector2f(p[1].x, p[1].y) - p0);
    p32 = (Eigen::Vector2f(p[2].x, p[2].y) - p3);
#endif
}

cv::Point2f Quad::interpolate(float x, float y) const
{
#ifdef INTERPOLATE
    Eigen::Vector2f r1 = p0 + p01 * (x + 1.) / 2.;
    Eigen::Vector2f r2 = p3 + p32 * (x + 1.) / 2.;
    Eigen::Vector2f r = r1 + (r2 - r1) * (y + 1) / 2;
    return {r(0), r(1)};
#else
    return homography.project(x, y);
#endif
}

cv::Point2f Quad::interpolate01(float x, float y) const
{
    return interpolate(2 * x - 1, 2 * y - 1);
}

void Quad::search(std::vector<Segment*>& path, Segment& parent, int depth,
                  std::vector<Quad>& quads, const cv::Point2f& opticalCenter)
{
    // cout << "Searching segment " << parent.getId() << ", depth=" << depth <<
    // ", #children=" << parent.children.size() << endl; terminal depth occurs
    // when we've found four segments.
    if (depth == 4) {
        // cout << "Entered terminal depth" << endl; // debug code

        // Is the first segment the same as the last segment (i.e., a loop?)
        if (path[4] == path[0]) {
            // the 4 corners of the quad as computed by the intersection of
            // segments.
            std::vector<cv::Point2f> p(4);
            float calculatedPerimeter = 0;
            bool bad = false;
            for (int i = 0; i < 4; i++) {
                // compute intersections between all the lines. This will give
                // us sub-pixel accuracy for the corners of the quad.
                GLine2D linea{path[i]->P0(), path[i]->P1()};
                GLine2D lineb{path[i + 1]->P0(), path[i + 1]->P1()};

                p[i] = linea.intersectionWith(lineb);
                calculatedPerimeter += path[i]->getLength();

                // no intersection? Occurs when the lines are almost parallel.
                if (p[i].x == -1)
                    bad = true;
            }

            // cout << "bad = " << bad << endl;
            // eliminate quads that don't form a simply connected loop, i.e.,
            // those that form an hour glass, or wind the wrong way.
            if (!bad) {
                float t0 = std::atan2(p[1].y - p[0].y, p[1].x - p[0].x);
                float t1 = std::atan2(p[2].y - p[1].y, p[2].x - p[1].x);
                float t2 = std::atan2(p[3].y - p[2].y, p[3].x - p[2].x);
                float t3 = std::atan2(p[0].y - p[3].y, p[0].x - p[3].x);

                //	double ttheta = fmod(t1-t0, 2*M_PI) + fmod(t2-t1, 2*M_PI) +
                //	  fmod(t3-t2, 2*M_PI) + fmod(t0-t3, 2*M_PI);
                float ttheta =
                    MathUtil::mod2pi(t1 - t0) + MathUtil::mod2pi(t2 - t1) +
                    MathUtil::mod2pi(t3 - t2) + MathUtil::mod2pi(t0 - t3);
                // cout << "ttheta=" << ttheta << endl;
                // the magic value is -2*PI. It should be exact,
                // but we allow for (lots of) numeric imprecision.
                if (ttheta < -7 || ttheta > -5)
                    bad = true;
            }

            if (!bad) {
                float d0 = cv::norm(p[0] - p[1]);
                float d1 = cv::norm(p[1] - p[2]);
                float d2 = cv::norm(p[2] - p[3]);
                float d3 = cv::norm(p[3] - p[0]);
                float d4 = cv::norm(p[0] - p[2]);
                float d5 = cv::norm(p[1] - p[3]);

                // check sizes
                if (d0 < Quad::minimumEdgeLength ||
                    d1 < Quad::minimumEdgeLength ||
                    d2 < Quad::minimumEdgeLength ||
                    d3 < Quad::minimumEdgeLength ||
                    d4 < Quad::minimumEdgeLength ||
                    d5 < Quad::minimumEdgeLength) {
                    bad = true;
                    // cout << "tagsize too small" << endl;
                }

                // check aspect ratio
                float dmax = std::max({d0, d1, d2, d3});
                float dmin = std::min({d0, d1, d2, d3});

                if (dmax > dmin * Quad::maxQuadAspectRatio) {
                    bad = true;
                    // cout << "aspect ratio too extreme" << endl;
                }
            }

            if (!bad) {
                Quad q(p, opticalCenter);
                q.segments = path;
                q.observedPerimeter = calculatedPerimeter;
                quads.push_back(q);
            }
        }
        return;
    }

    //  if (depth >= 1) // debug code
    // cout << "depth: " << depth << endl;

    // Not terminal depth. Recurse on any children that obey the correct
    // handedness.
    for (unsigned int i = 0; i < parent.children.size(); i++) {
        Segment& child = *parent.children[i];
        //    cout << "  Child " << child.getId() << ":  ";
        // (handedness was checked when we created the children)

        // we could rediscover each quad 4 times (starting from
        // each corner). If we had an arbitrary ordering over
        // points, we can eliminate the redundant detections by
        // requiring that the first corner have the lowest
        // value. We're arbitrarily going to use theta...
        if (child.getTheta() > path[0]->getTheta()) {
            // cout << "theta failed: " << child.getTheta() << " > " <<
            // path[0]->getTheta() << endl;
            continue;
        }
        path[depth + 1] = &child;
        search(path, child, depth + 1, quads, opticalCenter);
    }
}

} // namespace AprilTags
