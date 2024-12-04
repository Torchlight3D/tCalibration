#include "rectangle.h"

#include <format>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "common_types.h"
#include "utils.h"

namespace {

cv::Point2d ndiff(const cv::Point2d& a, const cv::Point2d& b)
{
    return cv::normalize(cv::Vec2d(a - b));
}

inline cv::Point2d perp(const cv::Point2d& a) { return {-a.y, a.x}; }

inline cv::Point2d avg(const cv::Point2d& a, const cv::Point2d& b)
{
    return (a + b) / 2.;
}

bool intersect(const cv::Point2d& p1, const cv::Point2d& d1,
               const cv::Point2d& p2, const cv::Point2d& d2, cv::Point2d& isect)
{
    double dk1 = p1.ddot(d1);
    double dk2 = p2.ddot(d2);

    double a1 = d1.x;
    double a2 = d2.x;
    double b1 = d1.y;
    double b2 = d2.y;

    double det = (a1 * b2 - a2 * b1);

    if (std::abs(det) < 1e-12) {
        // lines are actually parallel. this is impossible?
        LOG(INFO) << std::format("Warning: determinant near-zero: {}, {}",
                                 __FILE__, __LINE__);
        return false;
    }

    isect.x = (b2 * dk1 - b1 * dk2) / det;
    isect.y = (a1 * dk2 - a2 * dk1) / det;
    return true;
}

} // namespace

Mrectangle::Mrectangle()
    : thetas(4, .0),
      centroids(4, cv::Point2d{}),
      valid(true),
      corners(4, cv::Point2d(0.0, 0.0)),
      edges(4, cv::Point2d(0.0, 0.0)),
      normals(4, cv::Point2d(0.0, 0.0)),
      line_deviation(4, cv::Point3d(0.0, 0.0, 1.0))
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            quad_coeffs[i][j] = 0;
        }
    }
}

// build a rectangular buffer of "width" around midpoint of side k
Mrectangle::Mrectangle(const Mrectangle& b, size_t k, double width)
    : thetas(4, .0),
      centroids(4, cv::Point2d(0.0, 0.0)),
      valid(true),
      corners(4, cv::Point2d(0.0, 0.0)),
      edges(4, cv::Point2d(0.0, 0.0)),
      normals(4, cv::Point2d(0.0, 0.0)),
      boundary_length(0),
      line_deviation(4, cv::Point3d(0.0, 0.0, 1.0))
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            quad_coeffs[i][j] = 0;
        }
    }

    assert(k < b.centroids.size());
    assert(b.corners.size() == 4);
    assert(b.corner_map[k].size() == 2);

    const cv::Point2d& n = b.normals[k];
    cv::Point2d c1 = b.corners[b.corner_map[k][0]];
    cv::Point2d c2 = b.corners[b.corner_map[k][1]];

    cv::Point2d diff1 = cv::Point2d(c1.x - c2.x, c1.y - c2.y);
    double len = sqrt(diff1.ddot(diff1));
    cv::Point2d pn(-n.y, n.x);
    double e_adj = std::min(len * adjust, 4.0) / len;
    // TODO: this must probably be larger on wider PSFs
    e_adj = std::max(e_adj, 0.08);

    cv::Point2d ecent = avg(c1, c2);
#if 0
        Point2d delta(-pn.x*len*e_adj, -pn.y*len*e_adj);

        // probably do not need this anymore
        if (ndiff(ecent, c1).ddot(pn) > 0) {
            delta.x = -delta.x;
            delta.y = -delta.y;
        }

        c1.x += delta.x;
        c1.y += delta.y;
        c2.x -= delta.x;
        c2.y -= delta.y;

        vector<Point2d> new_corners(4);
        new_corners[0] = c1 + width*n;
        new_corners[1] = c1 - width*n;
        new_corners[2] = c2 + width*n;
        new_corners[3] = c2 - width*n;
#else
    double inset = std::max(5.0, std::min(16.0, len * 0.08));

    int ne = (k + 1) % 4;
    int pe = (k + 3) % 4;
    cv::Point2d nipn = b.centroids[ne] - inset * b.normals[ne];
    cv::Point2d nipe = b.centroids[pe] - inset * b.normals[pe];

    cv::Point2d si = b.centroids[k] - width * b.normals[k];
    cv::Point2d so = b.centroids[k] + width * b.normals[k];

    std::vector<cv::Point2d> new_corners(4);

    intersect(si, b.normals[k], nipn, b.normals[ne], new_corners[0]);
    intersect(si, b.normals[k], nipe, b.normals[pe], new_corners[1]);
    intersect(so, b.normals[k], nipn, b.normals[ne], new_corners[2]);
    intersect(so, b.normals[k], nipe, b.normals[pe], new_corners[3]);
#endif

    std::vector<std::pair<double, int>> orient(4);
    for (size_t i = 0; i < 4; i++) {
        cv::Point2d dv = new_corners[i] - ecent;
        orient[i] = std::make_pair(-atan2(dv.y, dv.x), int(i));
    }
    std::ranges::sort(orient);

    for (size_t i = 0; i < 4; i++) {
        corners[i] = new_corners[orient[i].second];
    }

    normals[0] = perp(ndiff(corners[0], corners[1]));
    normals[1] = perp(ndiff(corners[1], corners[2]));
    normals[2] = perp(ndiff(corners[2], corners[3]));
    normals[3] = perp(ndiff(corners[3], corners[0]));

    centroids[0] = avg(corners[0], corners[1]);
    centroids[1] = avg(corners[1], corners[2]);
    centroids[2] = avg(corners[2], corners[3]);
    centroids[3] = avg(corners[3], corners[0]);

    length = len;

    tl.x = 1e50;
    br.x = -1e50;
    tl.y = 1e50;
    br.y = -1e50;
    for (size_t k = 0; k < 4; k++) {
        const auto& c = corners[k];
        tl.x = std::min(tl.x, c.x);
        br.x = std::max(br.x, c.x);
        tl.y = std::min(tl.y, c.y);
        br.y = std::max(br.y, c.y);
    }

    tl.x = std::floor(tl.x);
    br.x = std::ceil(br.x);
    tl.y = std::floor(tl.y);
    br.y = std::ceil(br.y);
}

// reposition a rectangle using new estimates of the centroids and normals
Mrectangle::Mrectangle(const Mrectangle& b,
                       const std::vector<Edge_record>& edge_records)
    : thetas(4, 0.0),
      centroids(4, cv::Point2d(0.0, 0.0)),
      valid(true),
      corners(4, cv::Point2d(0.0, 0.0)),
      edges(4, cv::Point2d(0.0, 0.0)),
      normals(4, cv::Point2d(0.0, 0.0)),
      boundary_length(0),
      line_deviation(4, cv::Point3d(0.0, 0.0, 1.0))
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            quad_coeffs[i][j] = 0;
        }
    }

    cv::Point2d sq_centre(0, 0);
    for (int k = 0; k < 4; k++) {
        sq_centre += edge_records[k].centroid;
    }
    sq_centre *= 0.25;

    for (int k = 0; k < 4; k++) {
        centroids[k] = edge_records[k].centroid;
        normals[k] =
            cv::Point2d(cos(edge_records[k].angle), sin(edge_records[k].angle));
        cv::Point2d delta = centroids[k] - sq_centre;
        delta *= 1.0 / cv::norm(delta);
        double dot = normals[k].x * delta.x + normals[k].y * delta.y;
        // do we still want to flip the normals?
        if (dot < 0) {
            normals[k] = -normals[k];
        }
        edges[k].x = -normals[k].y; //??
        edges[k].y = normals[k].x;
        thetas[k] = atan2(normals[k].y, normals[k].x);
    }

    boundary_length = b.boundary_length;
    corner_map = std::vector<std::vector<int>>(4);

    int corner_idx = 0;
    std::vector<Intersection_record> isections;
    for (size_t k1 = 0; k1 < 3; k1++) {
        for (size_t k2 = k1 + 1; k2 < 4; k2++) {
            cv::Point2d isect{};
            if (intersect(centroids[k1], normals[k1], centroids[k2],
                          normals[k2], isect)) {
                double distance = std::min(cv::norm(centroids[k1] - isect),
                                           cv::norm(centroids[k2] - isect));
                isections.push_back(
                    Intersection_record(distance, isect, k1, k2));
            }
        }
    }

    // this cannot be a quad if we have fewer than 4 intersections
    if (isections.size() < 4) {
        valid = false;
        return;
    }
    // order intersections by distance from centroids; this eliminates the
    // false intersections
    std::sort(isections.begin(), isections.end());

    // now order the intersections on the clock face (relative to centroid)
    // to obtain a unique ordering
    cv::Point2d ccent(0, 0);
    for (size_t i = 0; i < 4; i++) {
        ccent += 0.25 * isections[i].intersection;
    }

    std::vector<std::pair<double, int>> orient(4);
    for (size_t i = 0; i < 4; i++) {
        cv::Point2d dv = isections[i].intersection - ccent;
        orient[i] = std::make_pair(-atan2(dv.y, dv.x), int(i));
    }
    std::ranges::sort(orient);

    // visit the corners in the winding order
    for (size_t i = 0; i < 4; i++) {
        corner_map[isections[orient[i].second].index1].push_back(corner_idx);
        corner_map[isections[orient[i].second].index2].push_back(corner_idx);
        corners[corner_idx++] = isections[orient[i].second].intersection;
    }

    tl.x = 1e50;
    br.x = -1e50;
    tl.y = 1e50;
    br.y = -1e50;
    for (size_t k = 0; k < 4; k++) {
        cv::Point2d& c = corners[k];
        if (c.x < tl.x)
            tl.x = c.x;
        if (c.x > br.x)
            br.x = c.x;
        if (c.y < tl.y)
            tl.y = c.y;
        if (c.y > br.y)
            br.y = c.y;
    }

    tl.x = floor(tl.x);
    br.x = ceil(br.x);
    tl.y = floor(tl.y);
    br.y = ceil(br.y);
}

Mrectangle::Mrectangle(const std::vector<double>& in_thetas,
                       const std::vector<double>& data_thetas,
                       const std::vector<cv::Point2d>& points,
                       const Gradient& g, int label, double thresh,
                       bool allow_partial)
    : thetas(in_thetas),
      centroids(4, cv::Point2d(0.0, 0.0)),
      valid(false),
      corners(4, cv::Point2d(0.0, 0.0)),
      edges(4, cv::Point2d(0.0, 0.0)),
      normals(4, cv::Point2d(0.0, 0.0)),
      corner_map(4),
      line_deviation(4, cv::Point3d(0.0, 0.0, 1.0))
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            quad_coeffs[i][j] = 0;
        }
    }

    if (thetas.size() == 4) {
        std::vector<double> c_weight(4, 0);
        for (size_t i = 0; i < points.size(); i++) {
            if (points[i].x <= 4 || points[i].x >= g.width() - 4 ||
                points[i].y <= 4 || points[i].y >= g.height() - 4) {
                if (!allow_partial) {
                    valid = false;
                    return;
                }
            }

            for (size_t k = 0; k < 4; k++) {
                if (tl::angular_diff(data_thetas[i], thetas[k]) < thresh) {
                    double w = g.grad_magnitude(points[i].x, points[i].y);
                    centroids[k].x += points[i].x * w;
                    centroids[k].y += points[i].y * w;
                    c_weight[k] += w;
                }
            }
        }

        for (size_t k = 0; k < 4; k++) {
            centroids[k].x /= c_weight[k];
            centroids[k].y /= c_weight[k];
        }

        for (size_t k = 0; k < 4; k++) {
            normals[k].x = cos(thetas[k]);
            normals[k].y = sin(thetas[k]);
            edges[k].x = -normals[k].y;
            edges[k].y = normals[k].x;
        }

        double long_side = 0;
        // now we have the normal to each edge, as well as a point on each
        // line

        // check if most points are closer than rect_it_thresh pixels from
        // any line
        std::vector<int> associated_edge(points.size(), -1);
        size_t outlier_count = 0;
        size_t severe_outlier_count = 0;
        for (size_t i = 0; i < points.size(); i++) {
            int min_edge = 0;
            double min_dist = 1e50;
            for (size_t k = 0; k < 4; k++) {
                const auto v = points[i] - centroids[k];
                double dist = std::abs(v.ddot(normals[k]));
                if (dist < min_dist) {
                    min_dist = dist;
                    min_edge = k;
                }
                min_dist = std::min(min_dist, dist);
            }
            if (min_dist > rect_il_thresh) {
                outlier_count++;
            }
            else {
                // an inlier that has the right orientation ...
                if (tl::angular_diff(data_thetas[i], thetas[min_edge]) <
                    4 * thresh) {
                    associated_edge[i] = min_edge;
                }
            }
        }
        size_t corner_idx = 0;
        if (outlier_count > points.size() / 8) {
            LOG(INFO) << std::format("label={}, outliers: {} out of {}", label,
                                     outlier_count, points.size());
            valid = false;
        }
        else {
            std::vector<int> inliers(4, 0);

            double covxx[4];
            double covxy[4];
            double covyy[4];
            for (size_t k = 0; k < 4; k++) {
                centroids[k] = cv::Point2d(0, 0);
                c_weight[k] = 0;
                covxx[k] = covxy[k] = covyy[k] = 0;
            }

            for (size_t i = 0; i < points.size(); i++) {
                if (associated_edge[i] >= 0) {
                    int& k = associated_edge[i];
                    inliers[k]++;
                    double w = g.grad_magnitude(points[i].x, points[i].y);

                    if (w > 0) {
                        double temp = w + c_weight[k];
                        cv::Point2d delta = points[i] - centroids[k];
                        cv::Point2d r = delta * (w / temp);
                        centroids[k] += r;

                        covxx[k] += c_weight[k] * delta.x * r.x;
                        covyy[k] += c_weight[k] * delta.y * r.y;
                        covxy[k] += c_weight[k] * delta.x * r.y;

                        c_weight[k] = temp;
                    }
                }
            }

            for (size_t k = 0; k < 4; k++) {
                covxx[k] /= c_weight[k];
                covxy[k] /= c_weight[k];
                covyy[k] /= c_weight[k];

                double tr = covxx[k] + covyy[k];
                double det = covxx[k] * covyy[k] - covxy[k] * covxy[k];

                double pa = 1.0;
                double pb = -tr;
                double pc = det;

                double q = -0.5 * (pb + sgn(pb) * sqrt(pb * pb - 4 * pa * pc));
                double l1 = q / pa;
                double l2 = pc / q;

                double l = std::max(l1, l2);
                long_side = std::max(long_side, l);

                // only update the normal if we have x-y correlation
                cv::Point2d ev;
                if (std::abs(covxy[k]) > 1e-10) {
                    ev.x = l - covyy[k];
                    ev.y = covxy[k];
                    ev *= 1.0 / norm(ev);
                    edges[k] = ev;
                    cv::Point2d nn(-edges[k].y, edges[k].x);
                    if (nn.ddot(normals[k]) < 0) {
                        normals[k] = -nn;
                    }
                    else {
                        normals[k] = nn;
                    }
                }
            }

            cv::Point2d extrema[4];
            for (size_t k = 0; k < 4; k++) {
                extrema[k] = cv::Point2d(1e15, -1e15);
            }

            for (size_t i = 0; i < points.size(); i++) {
                if (associated_edge[i] >= 0) {
                    int& k = associated_edge[i];

                    cv::Point2d delta = points[i] - centroids[k];
                    double par = delta.ddot(edges[k]);
                    extrema[k].x = std::min(extrema[k].x, par);
                    extrema[k].y = std::max(extrema[k].y, par);
                }
            }

            double x_scale[4];
            for (size_t k = 0; k < 4; k++) {
                // for some reason, no points were
                if (extrema[k].x > 0 || extrema[k].y < 0) {
                    // associated with this edge
                    x_scale[k] = 1.0;
                }
                else {
                    x_scale[k] =
                        std::abs(extrema[k].y - extrema[k].x) * 0.5 / sqrt(0.5);
                }

                // catch the case where only one point is associated with this
                // edge
                x_scale[k] = std::max(1.0, x_scale[k]);
            }

            // ok, so we have refined the edge centroid and normal
            // now we can fit a quadratic, and apply a stricter inlier test
            double D[4][3][3];
            double B[4][3];
            for (size_t k = 0; k < 4; k++) {
                for (int r = 0; r < 3; r++) {
                    B[k][r] = 0;
                    for (int c = 0; c < 3; c++) {
                        D[k][r][c] = 0;
                    }
                }
            }

            for (size_t i = 0; i < points.size(); i++) {
                if (associated_edge[i] >= 0) {
                    int& k = associated_edge[i];

                    cv::Point2d delta = points[i] - centroids[k];
                    double par = delta.ddot(edges[k]) / x_scale[k];
                    double perp = delta.ddot(normals[k]);

                    D[k][0][0] += 1;
                    D[k][0][1] += par;
                    D[k][0][2] += par * par;
                    D[k][1][2] += par * par * par;
                    D[k][2][2] += par * par * par * par;

                    B[k][0] += perp;
                    B[k][1] += par * perp;
                    B[k][2] += par * par * perp;
                }
            }

            // complete the design matrix D, and calculate quadratic
            // coefficients
            for (size_t k = 0; k < 4; k++) {
                D[k][1][0] = D[k][0][1];
                D[k][1][1] = D[k][0][2];
                D[k][2][0] = D[k][0][2];
                D[k][2][1] = D[k][1][2];

                // what do we do if the determinant is zero?

                double det_D =
                    D[k][0][0] *
                        (D[k][1][1] * D[k][2][2] - D[k][1][2] * D[k][2][1]) +
                    -D[k][0][1] *
                        (D[k][1][0] * D[k][2][2] - D[k][1][2] * D[k][2][0]) +
                    D[k][0][2] *
                        (D[k][1][0] * D[k][2][1] - D[k][1][1] * D[k][2][0]);

                if (std::abs(det_D) < 1e-12) {
                    valid = false;
                    return;
                }

                // replace column 0 with B
                double det_0 =
                    B[k][0] *
                        (D[k][1][1] * D[k][2][2] - D[k][1][2] * D[k][2][1]) +
                    -D[k][0][1] *
                        (B[k][1] * D[k][2][2] - D[k][1][2] * B[k][2]) +
                    D[k][0][2] * (B[k][1] * D[k][2][1] - D[k][1][1] * B[k][2]);

                // replace column 1 with B
                double det_1 =
                    D[k][0][0] * (B[k][1] * D[k][2][2] - D[k][1][2] * B[k][2]) +
                    -B[k][0] *
                        (D[k][1][0] * D[k][2][2] - D[k][1][2] * D[k][2][0]) +
                    D[k][0][2] * (D[k][1][0] * B[k][2] - B[k][1] * D[k][2][0]);

                // replace column 2 with B
                double det_2 =
                    D[k][0][0] * (D[k][1][1] * B[k][2] - B[k][1] * D[k][2][1]) +
                    -D[k][0][1] *
                        (D[k][1][0] * B[k][2] - B[k][1] * D[k][2][0]) +
                    B[k][0] *
                        (D[k][1][0] * D[k][2][1] - D[k][1][1] * D[k][2][0]);

                quad_coeffs[k][0] = det_2 / det_D / (x_scale[k] * x_scale[k]);
                quad_coeffs[k][1] = det_1 / det_D / x_scale[k];
                quad_coeffs[k][2] = det_0 / det_D;
            }

            outlier_count = 0;
            std::vector<int> point_count(4, 0);
            std::vector<int> inlier_count(4, 0);
            std::vector<cv::Point2d> min_recon(4, cv::Point2d(0, 1e20));
            std::vector<cv::Point2d> max_recon(4, cv::Point2d(0, -1e20));
            for (const auto& point : points) {
                int best_edge = 0;
                double min_err = 1e20;
                for (size_t k = 0; k < 4; k++) {
                    cv::Point2d delta = point - centroids[k];
                    double par = delta.ddot(edges[k]);
                    double recon_perp = quad_coeffs[k][0] * par * par +
                                        quad_coeffs[k][1] * par +
                                        quad_coeffs[k][2];
                    double err = std::abs(recon_perp - delta.ddot(normals[k]));
                    if (err < min_err) {
                        min_err = err;
                        best_edge = k;
                    }
                }
                if (min_err > quad_il_thresh) {
                    outlier_count++;
                    if (min_err > quad_severe_outlier_thresh) {
                        severe_outlier_count++;
                    }
                }

                cv::Point2d delta = point - centroids[best_edge];
                double par = delta.ddot(edges[best_edge]);
                double recon_perp = quad_coeffs[best_edge][0] * par * par +
                                    quad_coeffs[best_edge][1] * par +
                                    quad_coeffs[best_edge][2];

                if (recon_perp < min_recon[best_edge].y &&
                    min_err < quad_il_thresh) {
                    min_recon[best_edge] = cv::Point2d(par, recon_perp);
                }
                if (recon_perp > max_recon[best_edge].y &&
                    min_err < quad_il_thresh) {
                    max_recon[best_edge] = cv::Point2d(par, recon_perp);
                }
            }
            // so how many inliers should we have?
            double outlier_ratio =
                (outlier_count - severe_outlier_count) / double(points.size());
            double severe_outlier_ratio =
                severe_outlier_count / double(points.size());

            valid = true;
            if (severe_outlier_ratio > quad_severe_outlier_ratio_thresh ||
                outlier_ratio > quad_outlier_ratio_thresh) {
                LOG(INFO) << std::format("2nd round outliers: got {} points, "
                                         "ratio={}, severe ratio={}",
                                         points.size(), outlier_ratio,
                                         severe_outlier_ratio);
                valid = false;
            }

            int violation_count = 0;
            for (size_t k = 0; k < 4; k++) {
                const auto diff = max_recon[k] - min_recon[k];
                if (std::abs(diff.x) != 0) {
                    cv::Point3d deviation(0., std::abs(diff.y),
                                          std::abs(diff.x));
                    double slope = deviation.x = deviation.y / deviation.z;
                    line_deviation[k] = deviation;
                    if (slope > quad_slope_thresh) {
                        violation_count +=
                            slope > quad_severe_slope_thresh ? 2 : 1;
                    }
                }
            }

            // if the object is long, thin, and out of focus, we will allow it
            valid &= violation_count <= (long_side > 50 ? 2 : 0);
            if (!valid) {
                return;
            }

            boundary_length = points.size();
            std::vector<Intersection_record> isections;
            for (size_t k1 = 0; k1 < 3; k1++) {
                for (size_t k2 = k1 + 1; k2 < 4; k2++) {
                    // exclude near-parallel lines
                    if (std::abs(normals[k1].ddot(normals[k2])) < 0.86603) {
                        cv::Point2d isect{};
                        if (intersect(centroids[k1], normals[k1], centroids[k2],
                                      normals[k2], isect)) {
                            double distance =
                                std::min(cv::norm(centroids[k1] - isect),
                                         cv::norm(centroids[k2] - isect));
                            isections.push_back(
                                Intersection_record(distance, isect, k1, k2));
                        }
                    }
                }
            }
            // this cannot be a quad if we have
            if (isections.size() < 4) {
                // fewer than 4 intersections
                LOG(INFO) << std::format("label={}, #isections={}", label,
                                         isections.size());
                valid = false;
                return;
            }
            // order intersections by distance from centroids; this
            // eliminates the false intersections
            std::sort(isections.begin(), isections.end());

            // now order the intersections on the clock face (relative to
            // centroid) to obtain a unique ordering
            cv::Point2d ccent(0, 0);
            for (size_t i = 0; i < 4; i++) {
                ccent += 0.25 * isections[i].intersection;
            }

            std::vector<std::pair<double, int>> orient(4);
            for (size_t i = 0; i < 4; i++) {
                cv::Point2d dv = isections[i].intersection - ccent;
                orient[i] = std::make_pair(-atan2(dv.y, dv.x), int(i));
            }
            std::ranges::sort(orient);

            // visit the corners in the winding order
            for (size_t i = 0; i < 4; i++) {
                corner_map[isections[orient[i].second].index1].push_back(
                    corner_idx);
                corner_map[isections[orient[i].second].index2].push_back(
                    corner_idx);
                corners[corner_idx++] =
                    isections[orient[i].second].intersection;
            }
        }
        if (corner_idx < 4) {
            valid = false;
        }
        else {
            // check that the target is not too thin (e.g., minor axis < 14)
            double min_side = cv::norm(corners[0] - corners[1]);
            for (size_t k = 1; k < 4; k++) {
                min_side =
                    std::min(min_side, norm(corners[k] - corners[(k + 1) % 4]));
            }
            valid = min_side > minimum_object_width;

            // estimate area using the right triangle areas
            area = 0.5 * cv::norm(corners[0] - corners[1]) *
                       cv::norm(corners[2] - corners[1]) +
                   0.5 * cv::norm(corners[0] - corners[3]) *
                       cv::norm(corners[2] - corners[3]);

            tl.x = 1e50;
            br.x = -1e50;
            tl.y = 1e50;
            br.y = -1e50;
            for (size_t k = 0; k < 4; k++) {
                cv::Point2d& c = corners[k];
                if (c.x < tl.x)
                    tl.x = c.x;
                if (c.x > br.x)
                    br.x = c.x;
                if (c.y < tl.y)
                    tl.y = c.y;
                if (c.y > br.y)
                    br.y = c.y;
            }

            tl.x = floor(tl.x);
            br.x = ceil(br.x);
            tl.y = floor(tl.y);
            br.y = ceil(br.y);
        }
    }
    else {
        valid = false;
    }
}

bool Mrectangle::corners_ok() const
{
    if (corner_map.size() != 4) {
        return false;
    }

    bool ok = true;
    for (int k = 0; k < 4; k++) {
        ok &= corner_map[k].size() == 2;
    }
    return ok;
}

bool Mrectangle::is_inside(const cv::Point2d& p) const
{
    // a point is inside the rectangle if it falls along the positive
    // direction of each normal
    for (size_t k = 0; k < 4; k++) {
        if ((p - centroids[k]).ddot(normals[k]) < 0.) {
            return false;
        }
    }
    return true;
}

cv::Point2d Mrectangle::get_centroid(size_t i) const { return centroids[i]; }

void Mrectangle::print() const
{
    for (int k = 0; k < 4; k++) {
        // corner_map[k][0], corner_map[k][1]
        LOG(INFO) << std::format(
            "c(%lf, %lf), e(%lf, %lf), n(%lf, %lf), t(%lf), "
            "cr(%lf, %lf), map(%d, %d)",
            centroids[k].x, centroids[k].y, edges[k].x, edges[k].y,
            normals[k].x, normals[k].y, thetas[k], corners[k].x, corners[k].y,
            0, 0);
    }
}
