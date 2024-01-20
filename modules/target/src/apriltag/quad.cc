#include "quad.h"

#include <Eigen/Dense>

#include <tMath/MathBase>
#include "float_image.h"
#include "line_2d.h"

using namespace tl;

namespace apriltags {

namespace {
constexpr float kMaxAspectRatio{32}; //!< Use to prune quads with insane ratios.
constexpr int kMinEdgeLength{6};     //!< Measured along edges in pixel.
} // namespace

Quad::Quad(const std::array<cv::Point2f, 4>& corners,
           const cv::Point2f& opticalCenter)
    : quadPoints(corners),
      segments(),
      observedPerimeter(),
      homography(opticalCenter)
{
#ifdef STABLE_H
    std::array<cv::Point2f, 4> srcPts{
        cv::Point2f{-1.f, -1.f}, {1.f, -1.f}, {1.f, 1.f}, {-1.f, 1.f}};
    homography.setCorrespondences(srcPts.data(), 4, corners.data(), 4);
#else
    homography.addCorrespondence(-1, -1, quadPoints[0].first,
                                 quadPoints[0].second);
    homography.addCorrespondence(1, -1, quadPoints[1].first,
                                 quadPoints[1].second);
    homography.addCorrespondence(1, 1, quadPoints[2].first,
                                 quadPoints[2].second);
    homography.addCorrespondence(-1, 1, quadPoints[3].first,
                                 quadPoints[3].second);
#endif

#ifdef INTERPOLATE
    p0 = Eigen::Vector2f(corners[0].x, corners[0].y);
    p3 = Eigen::Vector2f(corners[3].x, corners[3].y);
    p01 = (Eigen::Vector2f(corners[1].x, corners[1].y) - p0);
    p32 = (Eigen::Vector2f(corners[2].x, corners[2].y) - p3);
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

void Quad::search(const FloatImage& fImage, std::vector<Segment*>& path,
                  Segment& parent, int depth, std::vector<Quad>& quads,
                  const cv::Point2f& opticalCenter)
{
    // cout << "Searching segment " << parent.getId() << ", depth=" << depth <<
    // ", #children=" << parent.children.size() << endl;

    // terminal depth occurs when we've found four segments.
    if (depth == 4) {
        // cout << "Entered terminal depth" << endl;

        // Is the first segment the same as the last segment (i.e., a loop?)
        if (path[4] == path[0]) {
            // the 4 corners of the quad as computed by the intersection of
            // segments.
            std::array<cv::Point2f, 4> corners{};
            float perimeter{0.f};
            bool bad{false};
            for (int i = 0; i < 4; i++) {
                // compute intersections between all the lines. This will give
                // us sub-pixel accuracy for the corners of the quad.
                GLine2D linea{path[i]->point0, path[i]->point1};
                GLine2D lineb{path[i + 1]->point0, path[i + 1]->point1};

                corners[i] = linea.intersectWith(lineb);
                perimeter += path[i]->length;

                // no intersection? Occurs when the lines are almost parallel.
                if (corners[i].x == -1) {
                    bad = true;
                }
            }
            // cout << "bad = " << bad << endl;

            // eliminate quads that don't form a simply connected loop, i.e.,
            // those that form an hour glass, or wind the wrong way.
            if (!bad) {
                float t0 = std::atan2(corners[1].y - corners[0].y,
                                      corners[1].x - corners[0].x);
                float t1 = std::atan2(corners[2].y - corners[1].y,
                                      corners[2].x - corners[1].x);
                float t2 = std::atan2(corners[3].y - corners[2].y,
                                      corners[3].x - corners[2].x);
                float t3 = std::atan2(corners[0].y - corners[3].y,
                                      corners[0].x - corners[3].x);

                //	double ttheta = fmod(t1-t0, 2*M_PI) + fmod(t2-t1, 2*M_PI) +
                //	  fmod(t3-t2, 2*M_PI) + fmod(t0-t3, 2*M_PI);
                float ttheta = math::mod2pi(t1 - t0) + math::mod2pi(t2 - t1) +
                               math::mod2pi(t3 - t2) + math::mod2pi(t0 - t3);
                // cout << "ttheta=" << ttheta << endl;
                // the magic value is -2*PI. It should be exact,
                // but we allow for (lots of) numeric imprecision.
                if (ttheta < -7 || ttheta > -5)
                    bad = true;
            }

            if (!bad) {
                float d0 = cv::norm(corners[0] - corners[1]);
                float d1 = cv::norm(corners[1] - corners[2]);
                float d2 = cv::norm(corners[2] - corners[3]);
                float d3 = cv::norm(corners[3] - corners[0]);
                float d4 = cv::norm(corners[0] - corners[2]);
                float d5 = cv::norm(corners[1] - corners[3]);

                // check sizes
                if (d0 < kMinEdgeLength || d1 < kMinEdgeLength ||
                    d2 < kMinEdgeLength || d3 < kMinEdgeLength ||
                    d4 < kMinEdgeLength || d5 < kMinEdgeLength) {
                    bad = true;
                    // cout << "tagsize too small" << endl;
                }

                // check aspect ratio
                float dmax = std::max(std::max(d0, d1), std::max(d2, d3));
                float dmin = std::min(std::min(d0, d1), std::min(d2, d3));

                if (dmax > dmin * kMaxAspectRatio) {
                    bad = true;
                    // cout << "aspect ratio too extreme" << endl;
                }
            }

            if (!bad) {
                Quad quad{corners, opticalCenter};
                quad.segments = path;
                quad.observedPerimeter = perimeter;

                quads.push_back(quad);
            }
        }

        return;
    }

    //  if (depth >= 1) // debug code
    // cout << "depth: " << depth << endl;

    // Not terminal depth. Recurse on any children that obey the correct
    // handedness.
    for (auto* child : parent.children) {
        //    cout << "  Child " << child.getId() << ":  ";
        // (handedness was checked when we created the children)

        // we could rediscover each quad 4 times (starting from
        // each corner). If we had an arbitrary ordering over
        // points, we can eliminate the redundant detections by
        // requiring that the first corner have the lowest
        // value. We're arbitrarily going to use theta...
        if (child->theta > path[0]->theta) {
            // cout << "theta failed: " << child.getTheta() << " > " <<
            // path[0]->getTheta() << endl;
            continue;
        }

        path[depth + 1] = child;
        search(fImage, path, *child, depth + 1, quads, opticalCenter);
    }
}

} // namespace apriltags
