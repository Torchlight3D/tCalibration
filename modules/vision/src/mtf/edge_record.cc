#include "edge_record.h"

#include "common_types.h"
#include "logger.h"

Edge_record::Edge_record() : pooled(false) {}

void Edge_record::pool_edges(Edge_record& a, Edge_record& b)
{
    Edge_record m;
    m.points.resize(a.points.size() + b.points.size());
    m.weights.resize(a.weights.size() + b.weights.size());

    for (size_t i = 0; i < a.points.size(); i++) {
        m.points[i].first = a.points[i].first - a.centroid.x;
        m.points[i].second = a.points[i].second - a.centroid.y;
        m.weights[i] = a.weights[i];
    }
    size_t as = a.points.size();
    for (size_t i = 0; i < b.points.size(); i++) {
        m.points[i + as].first = b.points[i].first - b.centroid.x;
        m.points[i + as].second = b.points[i].second - b.centroid.y;
        m.weights[i + as] = b.weights[i];
    }

    m.compute_eigenvector_angle();
    a.angle = m.angle;
    b.angle = m.angle;
    a.pooled = true;
    b.pooled = true;
}

bool Edge_record::compatible(const Edge_record& b)
{
    double Z;

    if (fabs(slope) > 2) {
        Z = (1.0 / slope - 1.0 / b.slope) / sqrt(sB * sB + b.sB * b.sB);
    }
    else {
        Z = (slope - b.slope) / sqrt(sB * sB + b.sB * b.sB);
    }

    return fabs(Z) <
           1.66; // ~90% confidence interval on t-distribution with ~80 dof
}

double Edge_record::relative_orientation(const Edge_record& b)
{
    cv::Point2d d1(cos(angle), sin(angle));
    cv::Point2d d2(cos(b.angle), sin(b.angle));

    return fabs(d1.x * d2.x + d1.y * d2.y);
}

void Edge_record::add_point(double x, double y, double gx, double gy)
{
    points.push_back(std::make_pair(x, y));
    double mag = (gx * gx + gy * gy);
    weights.push_back(mag);
}

std::pair<double, double> Edge_record::compute_eigenvector_angle()
{
    double covxx = 0;
    double covxy = 0;
    double covyy = 0;

    centroid.x = 0;
    centroid.y = 0;
    wsum = 0;
    for (size_t i = 0; i < points.size(); i++) {
        double w = weights[i];

        if (w > 0) {
            double temp = w + wsum;
            double delta_x = points[i].first - centroid.x;
            double delta_y = points[i].second - centroid.y;
            double rx = delta_x * w / temp;
            double ry = delta_y * w / temp;
            centroid.x += rx;
            centroid.y += ry;

            covxx += wsum * delta_x * rx;
            covyy += wsum * delta_y * ry;
            covxy += wsum * delta_x * ry;

            wsum = temp;
        }
    }

    covxx /= wsum;
    covxy /= wsum;
    covyy /= wsum;

    // char. poly: l^2 - (a+d)l + (ad-bc) = 0
    // thus l^2 -(tr)l + det = 0
    double tr = covxx + covyy;
    double det = covxx * covyy - covxy * covxy;

    double pa = 1.0;
    double pb = -tr;
    double pc = det;

    double q = -0.5 * (pb + sgn(pb) * sqrt(pb * pb - 4 * pa * pc));
    double l1 = q / pa;
    double l2 = pc / q;

    double l = std::max(l1, l2);
    assert(l >= 0);

    double ev[2];
    if (fabs(covxy) > 1e-10) {
        ev[0] = l - covyy;
        ev[1] = covxy;
        slope = ev[0] / ev[1]; // TODO: check this?
    }
    else {
        logger.info("%s\n", "Warning: edge is perfectly horizontal / vertical");
        if (covxx > covyy) {
            ev[0] = 1;
            ev[1] = 0;
        }
        else {
            ev[0] = 0;
            ev[1] = 1;
        }
        slope = 0;
    }

    angle = atan2(-ev[0], ev[1]);

    return std::make_pair(std::sqrt(std::max(fabs(l1), fabs(l2))),
                          std::sqrt(std::min(fabs(l1), fabs(l2))));
}

bool Edge_record::reduce()
{ // compute orientation, and remove weak points
    if (weights.size() <
        20) { // not enough points to really extract an edge here
        angle = 0;
        rsq = 1.0;
        return false;
    }

    // TODO: could look for peaks etc. here?

    renormalize_weights();
    std::vector<double> inweights(weights);
    std::pair<double, double> dims = compute_eigenvector_angle();
    cv::Point2d dir(cos(angle), sin(angle));

    std::vector<double> histo(2 * 16 * 8, 0);
    double total_weight = 0;

    for (size_t i = 0; i < points.size(); i++) {
        double dx = points[i].first - centroid.x;
        double dy = points[i].second - centroid.y;

        double dot = dx * dir.x + dy * dir.y;

        // dot is "across edge" direction
        // histogram of dot vs weight?
        double idot = lrint(dot * 8 + 16 * 8);
        if (idot >= 3 && idot < (int)histo.size() - 3) {
            histo[idot] += weights[i];
            histo[idot - 1] += weights[i];
            histo[idot + 1] += weights[i];
            histo[idot - 2] += weights[i];
            histo[idot + 2] += weights[i];
            histo[idot - 3] += weights[i];
            histo[idot + 3] += weights[i];

            total_weight += weights[i] * 7;
        }
    }
    // find peak value in central 5 pixel band
    size_t central_idx = 16 * 8;
    for (size_t j = (-5 * 8 + 16 * 8); j <= (5 * 8 + 16 * 8); j++) {
        if (histo[j] > histo[central_idx]) {
            central_idx = j;
        }
    }
    // now find 5% cut-off on both sides of peak
    size_t lower5p = central_idx - 1;
    while (lower5p > 1 && histo[lower5p] > 0.05 * histo[central_idx]) lower5p--;
    size_t upper5p = central_idx + 1;
    while (upper5p < histo.size() - 1 &&
           histo[upper5p] > 0.05 * histo[central_idx])
        upper5p++;

    // move outwards a little, then look for another value greater than the
    // 5% threshold
    size_t rise_lower5p = (size_t)std::max(int(1), int(lower5p) - 8);
    while (rise_lower5p > 1 && histo[rise_lower5p] <= histo[lower5p])
        rise_lower5p--;
    size_t rise_upper5p = std::min(histo.size() - 1, upper5p + 8);
    while (rise_upper5p < histo.size() - 1 &&
           histo[rise_upper5p] <= histo[upper5p])
        rise_upper5p++;

    double csum = 0;
    for (size_t i = 0; i < histo.size(); i++) {
        csum += histo[i];
        histo[i] = csum;
    }
    total_weight = csum;
    int p10idx = 0;
    for (size_t i = 1; i < histo.size(); i++) {
        if (fabs(histo[i] - 0.1 * csum) < fabs(histo[p10idx] - 0.1 * csum)) {
            p10idx = i;
        }
    }
    int p90idx = histo.size() - 1;
    for (size_t i = histo.size() - 2; i > 0; i--) {
        if (fabs(histo[i] - 0.9 * csum) < fabs(histo[p90idx] - 0.9 * csum)) {
            p90idx = i;
        }
    }

    double lower = p10idx / 8.0 - 16.0;
    double upper = p90idx / 8.0 - 16.0;
    double span = upper - lower;

    lower -= span * 0.7;
    upper += span * 0.7;

    lower = std::max(double(rise_lower5p + lower5p) / 16.0 - 16.0, lower);
    upper = std::min(double(rise_upper5p + upper5p) / 16.0 - 16.0, upper);

    // trim weights to 10%-90% region?
    for (size_t i = 0; i < points.size(); i++) {
        double dx = points[i].first - centroid.x;
        double dy = points[i].second - centroid.y;

        double dot = dx * dir.x + dy * dir.y;
        weights[i] = 0;
        if (dot >= lower && dot <= upper) {
            weights[i] = SQR(SQR(inweights[i])) * (1.0 / (10.0 + fabs(dot)));
        }
    }
    dims = compute_eigenvector_angle();

    radii = dims;

    sB = radii.second / (radii.first * sqrt(wsum));

    rsq = 0.0; // TODO: what is the appropriate measure of uncertainty in
               // the angle estimate?

    return true;
}

bool Edge_record::is_pooled() const { return pooled; }

void Edge_record::clear()
{
    weights.clear();
    points.clear();
}

void Edge_record::renormalize_weights()
{
    double maxw = 0;
    for (size_t i = 0; i < weights.size(); i++) {
        maxw = std::max(weights[i], maxw);
    }
    if (maxw > 0) {
        for (size_t i = 0; i < weights.size(); i++) {
            weights[i] /= maxw;
        }
    }
}
