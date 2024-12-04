#include "lsd.h"

#include <glog/logging.h>

#define M_3_2_PI (3 * M_PI) / 2 // 3/2 pi
#define M_2__PI (2 * M_PI)      // 2 pi
#define NOTDEF double(-1024.0)  // Label for pixels with undefined gradient.
#define NOTUSED 0               // Label for pixels not used in yet.
#define USED 1                  // Label for pixels already used in detection.

namespace tl {

namespace {

constexpr double DEG_TO_RADS{M_PI / 180};

// TODO: really want to make a loop class
template <typename T>
class Loop
{
public:
    using iterator = typename std::vector<T>::iterator;

    explicit Loop(const std::vector<T> &data) : _data(data), _it(_data.begin())
    {
    }

    const T &next() const { return *_it; }

private:
    std::vector<T> _data;
    iterator _it;
};

// Brief:
// Computes the natural logarithm of the absolute value of the gamma function of
// x using Windschitl method.
//
// Reference:
// See http://www.rskey.org/gamma.htm
inline double logGammaByWindschitl(double x)
{
    return 0.918938533204673 + (x - 0.5) * std::log(x) - x +
           0.5 * x *
               std::log(x * std::sinh(1 / x) + 1 / (810.0 * std::pow(x, 6.0)));
}

// Brief:
// Computes the natural logarithm of the absolute value of the gamma function of
// x using the Lanczos approximation.
//
// Reference:
// See http://www.rskey.org/gamma.htm
inline double logGammaByLanczos(double x)
{
    constexpr double q[]{75122.6331530, 80916.6278952, 36308.2951477,
                         8687.24529705, 1168.92649479, 83.8676043424,
                         2.50662827511};
    double a = (x + 0.5) * log(x + 5.5) - (x + 5.5);
    double b{0.};
    for (int n{0}; n < 7; ++n) {
        a -= std::log(x + double(n));
        b += q[n] * pow(x, double(n));
    }
    return a + log(b);
}

// TODO: use template<double x> after cpp20
#define log_gamma(x) \
    ((x) > 15.0 ? logGammaByWindschitl(x) : logGammaByLanczos(x))

inline double distSq(double x1, double y1, double x2, double y2)
{
    return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

inline double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(distSq(x1, y1, x2, y2));
}

// Signed angle difference
inline double angle_diff_signed(double a, double b)
{
    double diff = a - b;
    while (diff <= -M_PI) {
        diff += M_2__PI;
    }
    while (diff > M_PI) {
        diff -= M_2__PI;
    }
    return diff;
}

// Absolute value angle difference
inline double angle_diff(double a, double b)
{
    return std::abs(angle_diff_signed(a, b));
}

// Compare doubles by relative error.
// TODO: use isApprox
inline bool double_equal(double a, double b)
{
    if (a == b) {
        return true;
    }

    double abs_diff = std::abs(a - b);
    double abs_max = std::max(std::abs(a), std::abs(b));

    if (abs_max < DBL_MIN)
        abs_max = DBL_MIN;

    constexpr double kErrorFactor{100.};
    return (abs_diff / abs_max) <= (kErrorFactor * DBL_EPSILON);
}

} // namespace

bool LSDParams::isValid() const
{
    return scale > 0. && sigma_scale > 0. && quant >= 0. && ang_th > 0. &&
           ang_th < 180. && density_th >= 0. && density_th < 1. && n_bins > 0;
}

cv::Point2d LineSegmentDetector::RegionPoint::weightedPoint() const
{
    return cv::Point2d{toPoint()} * modgrad;
}

cv::Point LineSegmentDetector::RegionPoint::toPoint() const { return {x, y}; }

LineSegmentDetector::LineSegmentDetector(const LSDParams &params)
    : img_width_(0),
      img_height_(0),
      _LOG_NT(0),
      width_needed_(false),
      prec_needed_(false),
      nfa_needed_(false),
      params_(params)
{
    CV_Assert(params.isValid());
}

void LineSegmentDetector::detect(
    cv::Mat _image, std::vector<cv::Vec4f> &_lines,
    std::vector<std::vector<cv::Point>> &_line_points,
    std::vector<std::vector<int>> &_line_point_map, std::vector<double> &_width,
    std::vector<double> &_prec, std::vector<double> &_nfa)
{
    image_ = _image;
    CV_Assert(_image.empty() && _image.type() != CV_8UC1);

    width_needed_ = true;
    prec_needed_ = true;
    nfa_needed_ = false;

    _line_point_map.resize(_image.rows, std::vector<int>(_image.cols));
    std::vector<double> w, p, n;
    flsd(_lines, _line_points, _line_point_map, w, p, n);

    if (width_needed_)
        _width = w;
    if (prec_needed_)
        _prec = p;
    if (nfa_needed_)
        _nfa = n;

    // Clear used structures
    ordered_points_.clear();
}

void LineSegmentDetector::flsd(std::vector<cv::Vec4f> &lines,
                               std::vector<std::vector<cv::Point>> &line_points,
                               std::vector<std::vector<int>> &line_point_map,
                               std::vector<double> &widths,
                               std::vector<double> &precisions,
                               std::vector<double> &nfas)
{
    // Angle tolerance
    const double prec = M_PI * params_.ang_th / 180;
    const double p = params_.ang_th / 180;
    const double rho =
        params_.quant / sin(prec); // gradient magnitude threshold

    if (params_.scale != 1.) {
        cv::Mat gaussian_img;
        const double sigma = (params_.scale < 1.)
                                 ? (params_.sigma_scale / params_.scale)
                                 : params_.sigma_scale;
        constexpr double sprec{3.};
        const auto h = static_cast<unsigned int>(
            std::ceil(sigma * std::sqrt(2 * sprec * log(10.))));
        cv::Size ksize(1 + 2 * h, 1 + 2 * h); // kernel size
        cv::GaussianBlur(image_, gaussian_img, ksize, sigma);
        // Scale image to needed size
        cv::resize(gaussian_img, scaled_image_, cv::Size(), params_.scale,
                   params_.scale, cv::INTER_LINEAR_EXACT);
        ll_angle(rho, params_.n_bins);
    }
    else {
        scaled_image_ = image_;
        ll_angle(rho, params_.n_bins);
    }

    _LOG_NT =
        5 * (std::log10(double(img_width_)) + std::log10(double(img_height_))) /
            2 +
        std::log10(11.0);
    // minimal number of points in region that can give a meaningful event
    const size_t min_reg_size = size_t(-_LOG_NT / log10(p)) * 3;

    // // Initialize region only when needed
    // Mat region = Mat::zeros(scaled_image.size(), CV_8UC1);
    used_ = cv::Mat_<uchar>::zeros(scaled_image_.size()); // zeros = NOTUSED
    std::vector<RegionPoint> reg;

    // Search for line segments
    for (const auto &norm_point : ordered_points_) {
        const cv::Point &point = norm_point.p;
        if ((used_.at<uchar>(point.y, point.x) == NOTUSED) &&
            (angles_.at<double>(point.y, point.x) != NOTDEF)) {
            double reg_angle;
            regionGrow(point, prec, reg, reg_angle);

            // Ignore small regions
            if (reg.size() < min_reg_size) {
                continue;
            }

            // Construct rectangular approximation for the region
            Rect rec;
            regionToRect(reg, reg_angle, prec, p, rec);

            double log_nfa{-1.};
            if (params_.refine_mode > cv::LSD_REFINE_NONE) {
                // At least REFINE_STANDARD
                if (!refine(reg, reg_angle, prec, p, rec, params_.density_th)) {
                    continue;
                }

                if (params_.refine_mode >= cv::LSD_REFINE_ADV) {
                    // Compute NFA
                    log_nfa = rect_improve(rec, params_.log_eps);
                    if (log_nfa <= params_.log_eps) {
                        continue;
                    }
                }
            }
            // Found new line

            // Add the offset
            rec.x1 += 0.5;
            rec.y1 += 0.5;
            rec.x2 += 0.5;
            rec.y2 += 0.5;

            // Scale the result values if a sub-sampling was performed
            if (params_.scale != 1.) {
                rec.x1 /= params_.scale;
                rec.y1 /= params_.scale;
                rec.x2 /= params_.scale;
                rec.y2 /= params_.scale;
                rec.width /= params_.scale;
            }

            // Store the relevant data
            lines.push_back(cv::Vec4f(float(rec.x1), float(rec.y1),
                                      float(rec.x2), float(rec.y2)));

            if (width_needed_)
                widths.push_back(rec.width);
            if (prec_needed_)
                precisions.push_back(rec.p);
            if (nfa_needed_ && params_.refine_mode >= cv::LSD_REFINE_ADV)
                nfas.push_back(log_nfa);

            // NOTE: Extra statistic
            line_points.push_back(std::vector<cv::Point>());
            int line_index = line_points.size();
            for (const auto &reg_point : reg) {
                int x = int((reg_point.x + 0.5) / params_.scale);
                int y = int((reg_point.y + 0.5) / params_.scale);
                line_points[line_points.size() - 1].push_back(cv::Point(x, y));
                if (y < line_point_map.size() && x < line_point_map[0].size()) {
                    line_point_map[y][x] = line_index;
                }
            }
        }
    }
}

void LineSegmentDetector::get_edge_point(
    std::vector<RegionPoint> &region_points, double reg_angle,
    std::vector<cv::Point> &edge_points)
{
    edge_points.clear();
    std::sort(region_points.begin(), region_points.end(),
              [](const auto &pt1, const auto &pt2) { return pt1.x < pt2.x; });

    // Angle tolerance
    const double prec = M_PI * params_.ang_th / 180;
    const double p = params_.ang_th / 180;
    const double rho =
        params_.quant / std::sin(prec); // gradient magnitude threshold

    constexpr int kGap{20};
    for (int i = 0; i < region_points.size() - kGap; i += kGap) {
        Rect rec;
        regionToRect(std::vector<RegionPoint>(region_points.begin() + i,
                                              region_points.begin() + i + kGap),
                     reg_angle, prec, p, rec);
        rec.x1 += 0.5;
        rec.y1 += 0.5;
        rec.x2 += 0.5;
        rec.y2 += 0.5;
        edge_points.push_back(cv::Point(int(rec.x1 / params_.scale),
                                        int(rec.y1 / params_.scale)));
        edge_points.push_back(cv::Point(int(rec.x2 / params_.scale),
                                        int(rec.y2 / params_.scale)));
    }
}

void LineSegmentDetector::ll_angle(double threshold, unsigned int n_bins)
{
    // Initialize data
    angles_ = cv::Mat_<double>(scaled_image_.size());
    modgrad_ = cv::Mat_<double>(scaled_image_.size());

    img_width_ = scaled_image_.cols;
    img_height_ = scaled_image_.rows;

    // Undefined the down and right boundaries
    angles_.row(img_height_ - 1).setTo(NOTDEF);
    angles_.col(img_width_ - 1).setTo(NOTDEF);

    // Computing gradient for remaining pixels
    double max_grad{-1.};
    for (int y = 0; y < img_height_ - 1; ++y) {
        const auto *scaled_image_row = scaled_image_.ptr<uchar>(y);
        const auto *next_scaled_image_row = scaled_image_.ptr<uchar>(y + 1);
        auto *angles_row = angles_.ptr<double>(y);
        auto *modgrad_row = modgrad_.ptr<double>(y);
        for (int x = 0; x < img_width_ - 1; ++x) {
            int DA = next_scaled_image_row[x + 1] - scaled_image_row[x];
            int BC = scaled_image_row[x + 1] - next_scaled_image_row[x];
            int gx = DA + BC; // gradient x component
            int gy = DA - BC; // gradient y component
            double norm = std::sqrt((gx * gx + gy * gy) / 4.0); // gradient norm

            modgrad_row[x] = norm; // store gradient

            if (norm <= threshold) // norm too small, gradient no defined
            {
                angles_row[x] = NOTDEF;
            }
            else {
                // gradient angle computation
                angles_row[x] =
                    cv::fastAtan2(float(gx), float(-gy)) * DEG_TO_RADS;
                if (norm > max_grad) {
                    max_grad = norm;
                }
            }
        }
    }

    // Compute histogram of gradient values
    // If all image is smooth, max_grad <= 0
    double bin_coef = (max_grad > 0.) ? double(n_bins - 1) / max_grad : 0.;
    for (int y = 0; y < img_height_ - 1; ++y) {
        const auto *modgrad_row = modgrad_.ptr<double>(y);
        for (int x = 0; x < img_width_ - 1; ++x) {
            NormPoint _point;
            int i = int(modgrad_row[x] * bin_coef);
            _point.p = cv::Point(x, y);
            _point.norm = i;
            ordered_points_.push_back(_point);
        }
    }

    std::sort(
        ordered_points_.begin(), ordered_points_.end(),
        [](const auto &pt1, const auto &pt2) { return pt1.norm > pt2.norm; });
}

void LineSegmentDetector::regionGrow(const cv::Point &s, double prec,
                                     std::vector<RegionPoint> &region_points,
                                     double &reg_angle)
{
    region_points.clear();

    // Point to this region
    RegionPoint seed;
    seed.x = s.x;
    seed.y = s.y;
    seed.used = &used_.at<uchar>(s.y, s.x);
    reg_angle = angles_.at<double>(s.y, s.x);
    seed.angle = reg_angle;
    seed.modgrad = modgrad_.at<double>(s.y, s.x);
    region_points.push_back(seed);

    float sumdx = float(std::cos(reg_angle));
    float sumdy = float(std::sin(reg_angle));
    *seed.used = USED;

    // Try neighboring regions
    for (const auto &rpoint : region_points) {
        int xx_min = std::max(rpoint.x - 1, 0);
        int xx_max = std::min(rpoint.x + 1, img_width_ - 1);
        int yy_min = std::max(rpoint.y - 1, 0);
        int yy_max = std::min(rpoint.y + 1, img_height_ - 1);
        for (int yy = yy_min; yy <= yy_max; ++yy) {
            auto *used_row = used_.ptr<uchar>(yy);
            const auto *angles_row = angles_.ptr<double>(yy);
            const auto *modgrad_row = modgrad_.ptr<double>(yy);
            for (int xx = xx_min; xx <= xx_max; ++xx) {
                uchar &is_used = used_row[xx];
                if (is_used != USED && (isAligned(xx, yy, reg_angle, prec))) {
                    const double &angle = angles_row[xx];
                    // Add point
                    is_used = USED;
                    RegionPoint region_point;
                    region_point.x = xx;
                    region_point.y = yy;
                    region_point.used = &is_used;
                    region_point.modgrad = modgrad_row[xx];
                    region_point.angle = angle;
                    region_points.push_back(region_point);

                    // Update region's angle
                    sumdx += cos(float(angle));
                    sumdy += sin(float(angle));
                    // reg_angle is used in the isAligned, so it needs to be
                    // updates?
                    reg_angle = cv::fastAtan2(sumdy, sumdx) * DEG_TO_RADS;
                }
            }
        }
    }
}

void LineSegmentDetector::regionToRect(
    const std::vector<RegionPoint> &region_points, double reg_angle,
    double prec, double p, Rect &rec) const
{
    double x = 0, y = 0, sum = 0;
    for (const auto &pt : region_points) {
        const double &weight = pt.modgrad;
        x += double(pt.x) * weight;
        y += double(pt.y) * weight;
        sum += weight;
    }

    // Weighted sum must differ from 0
    CV_Assert(sum > 0);

    x /= sum;
    y /= sum;

    double theta = get_theta(region_points, x, y, reg_angle, prec);

    // Find length and width
    double dx = cos(theta);
    double dy = sin(theta);
    double l_min = 0, l_max = 0, w_min = 0, w_max = 0;

    for (const auto &pt : region_points) {
        double regdx = double(pt.x) - x;
        double regdy = double(pt.y) - y;

        double l = regdx * dx + regdy * dy;
        double w = -regdx * dy + regdy * dx;

        if (l > l_max)
            l_max = l;
        else if (l < l_min)
            l_min = l;
        if (w > w_max)
            w_max = w;
        else if (w < w_min)
            w_min = w;
    }

    // Store values
    rec.x1 = x + l_min * dx;
    rec.y1 = y + l_min * dy;
    rec.x2 = x + l_max * dx;
    rec.y2 = y + l_max * dy;
    rec.width = w_max - w_min;
    rec.x = x;
    rec.y = y;
    rec.theta = theta;
    rec.dx = dx;
    rec.dy = dy;
    rec.prec = prec;
    rec.p = p;

    // Min width of 1 pixel
    rec.width = std::max(rec.width, 1.);
}

double LineSegmentDetector::get_theta(
    const std::vector<RegionPoint> &region_points, double x, double y,
    double reg_angle, double prec) const
{
    double Ixx = 0.0;
    double Iyy = 0.0;
    double Ixy = 0.0;

    // Compute inertia matrix
    for (const auto &rpoint : region_points) {
        const double &regx = rpoint.x;
        const double &regy = rpoint.y;
        const double &weight = rpoint.modgrad;
        double dx = regx - x;
        double dy = regy - y;
        Ixx += dy * dy * weight;
        Iyy += dx * dx * weight;
        Ixy -= dx * dy * weight;
    }

    // Check if inertia matrix is null
    CV_Assert(!double_equal(Ixx, 0) || !double_equal(Iyy, 0) ||
              !double_equal(Ixy, 0));

    // Compute smallest eigenvalue
    double lambda =
        0.5 *
        (Ixx + Iyy - std::sqrt((Ixx - Iyy) * (Ixx - Iyy) + 4.0 * Ixy * Ixy));

    // Compute angle
    double theta =
        (std::abs(Ixx) > std::abs(Iyy))
            ? double(cv::fastAtan2(float(lambda - Ixx), float(Ixy)))
            : double(cv::fastAtan2(float(Ixy), float(lambda - Iyy))); // in degs
    theta *= DEG_TO_RADS;

    // Correct angle by 180 deg if necessary
    if (angle_diff(theta, reg_angle) > prec) {
        theta += M_PI;
    }

    return theta;
}

bool LineSegmentDetector::refine(std::vector<RegionPoint> &region_points,
                                 double reg_angle, double prec, double p,
                                 Rect &rec, double density_th)
{
    double density = region_points.size() /
                     (dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);

    if (density >= density_th) {
        return true;
    }

    // Try to reduce angle tolerance
    double xc = double(region_points[0].x);
    double yc = double(region_points[0].y);
    const double &ang_c = region_points[0].angle;
    double sum = 0, s_sum = 0;
    int n = 0;

    for (const auto &rpoint : region_points) {
        *(rpoint.used) = NOTUSED;
        if (dist(xc, yc, rpoint.x, rpoint.y) < rec.width) {
            const double &angle = rpoint.angle;
            double ang_d = angle_diff_signed(angle, ang_c);
            sum += ang_d;
            s_sum += ang_d * ang_d;
            ++n;
        }
    }

    CV_Assert(n > 0);

    double mean_angle = sum / double(n);
    // 2 * standard deviation
    double tau = 2.0 * sqrt((s_sum - 2.0 * mean_angle * sum) / double(n) +
                            mean_angle * mean_angle);

    // Try new region
    regionGrow(cv::Point(region_points[0].x, region_points[0].y), tau,
               region_points, reg_angle);
    if (region_points.size() < 2) {
        return false;
    }

    regionToRect(region_points, reg_angle, prec, p, rec);
    density = region_points.size() /
              (dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);

    if (density < density_th) {
        return reduceRegionRadius(region_points, reg_angle, prec, p, rec,
                                  density, density_th);
    }

    return true;
}

bool LineSegmentDetector::reduceRegionRadius(std::vector<RegionPoint> &reg,
                                             double reg_angle, double prec,
                                             double p, Rect &rec,
                                             double density, double density_th)
{
    // Compute region's radius
    double xc = double(reg[0].x);
    double yc = double(reg[0].y);
    double radSq1 = distSq(xc, yc, rec.x1, rec.y1);
    double radSq2 = distSq(xc, yc, rec.x2, rec.y2);
    double radSq = radSq1 > radSq2 ? radSq1 : radSq2;

    while (density < density_th) {
        constexpr double reduce_rate{0.75};
        radSq *= reduce_rate * reduce_rate;
        // Remove points from the region and update 'used' map
        for (size_t i = 0; i < reg.size(); ++i) {
            if (distSq(xc, yc, double(reg[i].x), double(reg[i].y)) > radSq) {
                // Remove point from the region
                *(reg[i].used) = NOTUSED;
                std::swap(reg[i], reg[reg.size() - 1]);
                reg.pop_back();
                --i; // To avoid skipping one point
            }
        }

        if (reg.size() < 2) {
            return false;
        }

        // Re-compute rectangle
        regionToRect(reg, reg_angle, prec, p, rec);

        // Re-compute region points density
        density = double(reg.size()) /
                  (dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);
    }

    return true;
}

double LineSegmentDetector::rect_improve(Rect &rect, double log_eps) const
{
    constexpr double delta{0.5};
    constexpr double delta_2 = delta / 2.0;

    double log_nfa = rect_nfa(rect);

    // Good rectangle
    if (log_nfa > log_eps) {
        return log_nfa;
    }

    // Try to improve
    // Finer precision
    Rect r = rect;
    for (int n = 0; n < 5; ++n) {
        r.p /= 2;
        r.prec = r.p * M_PI;
        double log_nfa_new = rect_nfa(r);
        if (log_nfa_new > log_nfa) {
            log_nfa = log_nfa_new;
            rect = Rect(r);
        }
    }
    if (log_nfa > log_eps) {
        return log_nfa;
    }

    // Try to reduce width
    r = Rect(rect);
    for (int n{0}; n < 5; ++n) {
        if ((r.width - delta) >= 0.5) {
            r.width -= delta;
            double log_nfa_new = rect_nfa(r);
            if (log_nfa_new > log_nfa) {
                rect = Rect(r);
                log_nfa = log_nfa_new;
            }
        }
    }
    if (log_nfa > log_eps) {
        return log_nfa;
    }

    // Try to reduce one side of rectangle
    r = Rect(rect);
    for (int n{0}; n < 5; ++n) {
        if ((r.width - delta) >= 0.5) {
            r.x1 += -r.dy * delta_2;
            r.y1 += r.dx * delta_2;
            r.x2 += -r.dy * delta_2;
            r.y2 += r.dx * delta_2;
            r.width -= delta;
            double log_nfa_new = rect_nfa(r);
            if (log_nfa_new > log_nfa) {
                rect = Rect(r);
                log_nfa = log_nfa_new;
            }
        }
    }
    if (log_nfa > log_eps) {
        return log_nfa;
    }

    // Try to reduce other side of rectangle
    r = Rect(rect);
    for (int n{0}; n < 5; ++n) {
        if ((r.width - delta) >= 0.5) {
            r.x1 -= -r.dy * delta_2;
            r.y1 -= r.dx * delta_2;
            r.x2 -= -r.dy * delta_2;
            r.y2 -= r.dx * delta_2;
            r.width -= delta;
            double log_nfa_new = rect_nfa(r);
            if (log_nfa_new > log_nfa) {
                rect = Rect(r);
                log_nfa = log_nfa_new;
            }
        }
    }
    if (log_nfa > log_eps) {
        return log_nfa;
    }

    // Try finer precision
    r = Rect(rect);
    for (int n{0}; n < 5; ++n) {
        if ((r.width - delta) >= 0.5) {
            r.p /= 2;
            r.prec = r.p * M_PI;
            double log_nfa_new = rect_nfa(r);
            if (log_nfa_new > log_nfa) {
                rect = Rect(r);
                log_nfa = log_nfa_new;
            }
        }
    }

    return log_nfa;
}

struct Edge
{
    cv::Point2f p;
    bool taken;
};

bool LineSegmentDetector::isAligned(int x, int y, double theta,
                                    double prec) const
{
    if (x < 0 || y < 0 || x >= angles_.cols || y >= angles_.rows) {
        return false;
    }

    const double &a = angles_.at<double>(y, x);
    if (a == NOTDEF) {
        return false;
    }

    // It is assumed that 'theta' and 'a' are in the range [-pi,pi]
    double n_theta = theta - a;
    if (n_theta < 0) {
        n_theta = -n_theta;
    }
    if (n_theta > M_3_2_PI) {
        n_theta -= M_2__PI;
        if (n_theta < 0)
            n_theta = -n_theta;
    }

    return n_theta <= prec;
}

void LineSegmentDetector::drawSegments(
    cv::Mat &image, const std::vector<std::vector<cv::Point>> &segments)
{
    CV_Assert(!image.empty());

    if (image_.channels() == 1) {
        cv::cvtColor(image_, image_, cv::COLOR_GRAY2BGR);
    }

    bool toggle{true};
    for (const auto &seg : segments) {
        auto color = toggle ? cv::Scalar(0, 55, 255) : cv::Scalar(155, 255, 50);
        toggle = !toggle;

        for (const auto &pt : seg) {
            cv::circle(image_, pt, 2, color, 2);
        }
    }
}

void LineSegmentDetector::drawSegments(cv::Mat &image,
                                       const std::vector<cv::Vec4f> &lines)
{
    CV_Assert(!image.empty());

    if (image.channels() == 1) {
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    }

    bool toggle{true};
    for (const auto &line : lines) {
        auto color = toggle ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        toggle = !toggle;

        const cv::Point2f b{line[0], line[1]};
        const cv::Point2f e{line[2], line[3]};
        cv::line(image, b, e, color, 1);
    }
}

double LineSegmentDetector::rect_nfa(const Rect &rec) const
{
    int total_pts = 0, alg_pts = 0;
    double half_width = rec.width / 2.0;
    double dyhw = rec.dy * half_width;
    double dxhw = rec.dx * half_width;

    Edge ordered_x[4];
    Edge *min_y = &ordered_x[0];
    Edge *max_y = &ordered_x[0]; // Will be used for loop range

    ordered_x[0].p.x = int(rec.x1 - dyhw);
    ordered_x[0].p.y = int(rec.y1 + dxhw);
    ordered_x[0].taken = false;
    ordered_x[1].p.x = int(rec.x2 - dyhw);
    ordered_x[1].p.y = int(rec.y2 + dxhw);
    ordered_x[1].taken = false;
    ordered_x[2].p.x = int(rec.x2 + dyhw);
    ordered_x[2].p.y = int(rec.y2 - dxhw);
    ordered_x[2].taken = false;
    ordered_x[3].p.x = int(rec.x1 + dyhw);
    ordered_x[3].p.y = int(rec.y1 - dxhw);
    ordered_x[3].taken = false;

    std::sort(ordered_x, ordered_x + 4, [](const Edge &a, const Edge &b) {
        return (a.p.x == b.p.x) ? (a.p.y < b.p.y) : (a.p.x < b.p.x);
    });

    // Find min y. And mark as taken. find max y.
    for (int i{1}; i < 4; ++i) {
        if (min_y->p.y > ordered_x[i].p.y) {
            min_y = &ordered_x[i];
        }
        if (max_y->p.y < ordered_x[i].p.y) {
            max_y = &ordered_x[i];
        }
    }
    min_y->taken = true;

    // Find leftmost untaken point.
    Edge *leftmost{nullptr};
    for (int i{0}; i < 4; ++i) {
        if (!ordered_x[i].taken) {
            if (!leftmost) // if uninitialized
            {
                leftmost = &ordered_x[i];
            }
            else if (leftmost->p.x > ordered_x[i].p.x) {
                leftmost = &ordered_x[i];
            }
        }
    }
    CV_Assert(leftmost);
    leftmost->taken = true;

    // Find rightmost untaken point.
    Edge *rightmost{nullptr};
    for (int i{0}; i < 4; ++i) {
        if (!ordered_x[i].taken) {
            if (!rightmost) // if uninitialized
            {
                rightmost = &ordered_x[i];
            }
            else if (rightmost->p.x < ordered_x[i].p.x) {
                rightmost = &ordered_x[i];
            }
        }
    }
    CV_Assert(rightmost);
    rightmost->taken = true;

    // Find last untaken point;
    Edge *tailp{nullptr};
    for (int i{0}; i < 4; ++i) {
        if (!ordered_x[i].taken) {
            if (!tailp) // if uninitialized
            {
                tailp = &ordered_x[i];
            }
            else if (tailp->p.x > ordered_x[i].p.x) {
                tailp = &ordered_x[i];
            }
        }
    }
    CV_Assert(tailp);
    tailp->taken = true;

    // first left step
    double flstep =
        (min_y->p.y != leftmost->p.y)
            ? (min_y->p.x - leftmost->p.x) / (min_y->p.y - leftmost->p.y)
            : 0.;
    // second left step
    double slstep =
        (leftmost->p.y != tailp->p.x)
            ? (leftmost->p.x - tailp->p.x) / (leftmost->p.y - tailp->p.x)
            : 0.;

    // first right step
    double frstep =
        (min_y->p.y != rightmost->p.y)
            ? (min_y->p.x - rightmost->p.x) / (min_y->p.y - rightmost->p.y)
            : 0.;
    // second right step
    double srstep =
        (rightmost->p.y != tailp->p.x)
            ? (rightmost->p.x - tailp->p.x) / (rightmost->p.y - tailp->p.x)
            : 0.;

    double lstep = flstep;
    double rstep = frstep;

    double left_x = min_y->p.x;
    double right_x = min_y->p.x;

    // Loop around all points in the region and count those that are aligned.
    int min_iter = min_y->p.y;
    int max_iter = max_y->p.y;
    for (int y = min_iter; y <= max_iter; ++y) {
        if (y < 0 || y >= img_height_) {
            continue;
        }

        for (int x = int(left_x); x <= int(right_x); ++x) {
            if (x < 0 || x >= img_width_) {
                continue;
            }

            ++total_pts;
            if (isAligned(x, y, rec.theta, rec.prec)) {
                ++alg_pts;
            }
        }

        if (y >= leftmost->p.y) {
            lstep = slstep;
        }

        if (y >= rightmost->p.y) {
            rstep = srstep;
        }

        left_x += lstep;
        right_x += rstep;
    }

    return nfa(total_pts, alg_pts, rec.p);
}

double LineSegmentDetector::nfa(int n, int k, double p) const
{
    // Trivial cases
    if (n == 0 || k == 0) {
        return -_LOG_NT;
    }
    if (n == k) {
        return -_LOG_NT - double(n) * std::log10(p);
    }

    double p_term = p / (1 - p);

    double log1term = (double(n) + 1) - log_gamma(double(k) + 1) -
                      log_gamma(double(n - k) + 1) + double(k) * log(p) +
                      double(n - k) * log(1.0 - p);
    double term = exp(log1term);

    if (double_equal(term, 0)) {
        if (k > n * p)
            return -log1term / M_LN10 - _LOG_NT;

        return -_LOG_NT;
    }

    // Compute more terms if needed

    // Error of 10% in the result is accepted
    constexpr double kTolerance = 0.1;
    double bin_tail = term;
    for (int i = k + 1; i <= n; ++i) {
        double bin_term = double(n - i + 1) / i;
        double mult_term = bin_term * p_term;
        term *= mult_term;
        bin_tail += term;
        if (bin_term < 1) {
            double err = term * ((1 - std::pow(mult_term, double(n - i + 1))) /
                                     (1 - mult_term) -
                                 1);
            if (err < kTolerance * std::fabs(-std::log10(bin_tail) - _LOG_NT) *
                          bin_tail) {
                break;
            }
        }
    }
    return -std::log10(bin_tail) - _LOG_NT;
}

} // namespace tl
