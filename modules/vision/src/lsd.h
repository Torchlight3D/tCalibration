#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <tCore/tGlobal>

namespace tl {

struct LSDParams
{
    double scale = 0.8;

    // Sigma for Gaussian filter is computed as sigma = sigma_scale / scale
    double sigma_scale = 0.6;

    // Bound to the quantization error on the gradient norm
    double quant = 2.0;

    // Gradient angle tolerance, in degrees.
    double ang_th = 22.5;

    // Detection threshold: -log10(NFA) > log_eps
    double log_eps = 0.;

    // Minimal density of region points in rectangle.
    double density_th = 0.7;

    int refine_mode = cv::LSD_REFINE_NONE;

    // Number of bins in pseudo-ordering of gradient modulus.
    int n_bins = 1024;

    LSDParams() {}

    bool isValid() const;
};

class LineSegmentDetector
{
public:
    LineSegmentDetector(const LSDParams &params = {});

    void detect(cv::Mat image, std::vector<cv::Vec4f> &lines,
                std::vector<std::vector<cv::Point>> &linePoints,
                std::vector<std::vector<int>> &line_point_map,
                std::vector<double> &width, std::vector<double> &prec,
                std::vector<double> &nfas);

    void drawSegments(cv::Mat &image,
                      const std::vector<std::vector<cv::Point>> &line_points);
    void drawSegments(cv::Mat &image, const std::vector<cv::Vec4f> &lines);

private:
    DISABLE_COPY(LineSegmentDetector)

    struct RegionPoint
    {
        int x;
        int y;
        uchar *used;
        double angle;
        double modgrad;

        cv::Point2d weightedPoint() const;
        cv::Point toPoint() const;
    };

    struct NormPoint
    {
        cv::Point p;
        int norm;
    };

    struct Rect
    {
        double x1, y1, x2, y2; // first and second point of the line segment
        double width;          // rectangle width
        double x, y;           // center of the rectangle
        double theta;          // angle
        double dx, dy;         // (dx,dy) is vector oriented as the line segment
        double prec;           // tolerance angle
        double p; // probability of a point with angle within 'prec'
    };

    void flsd(std::vector<cv::Vec4f> &lines,
              std::vector<std::vector<cv::Point>> &line_points,
              std::vector<std::vector<int>> &line_point_map,
              std::vector<double> &widths, std::vector<double> &precisions,
              std::vector<double> &nfas);

    // NOTE: Not used yet
    void get_edge_point(std::vector<RegionPoint> &reg_points, double reg_angle,
                        std::vector<cv::Point> &edge_point);

    void ll_angle(double threshold, unsigned int n_bins);

    void regionGrow(const cv::Point &s, double prec,
                    std::vector<RegionPoint> &reg, double &reg_angle);

    void regionToRect(const std::vector<RegionPoint> &region, double reg_angle,
                      double, double p, Rect &rec) const;

    double get_theta(const std::vector<RegionPoint> &region, double x, double y,
                     double reg_angle, double prec) const;

    bool refine(std::vector<RegionPoint> &reg, double reg_angle, double prec,
                double p, Rect &rec, double density_th);

    bool reduceRegionRadius(std::vector<RegionPoint> &reg, double reg_angle,
                            double prec, double p, Rect &rec, double density,
                            double density_th);

    // Try new rect variations to improve NFA values.
    double rect_improve(Rect &rec, double log_eps) const;

    double rect_nfa(const Rect &rec) const;

    double nfa(int n, int k, double p) const;

    bool isAligned(int x, int y, double theta, double prec) const;

private:
    cv::Mat image_;
    cv::Mat scaled_image_;
    cv::Mat_<double> angles_; // in rads
    cv::Mat_<double> modgrad_;
    cv::Mat_<uchar> used_;

    std::vector<NormPoint> ordered_points_;

    int img_width_;
    int img_height_;
    double _LOG_NT;

    bool width_needed_;
    bool prec_needed_;
    bool nfa_needed_;

    const LSDParams params_;
};

} // namespace tl
