#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

class Undistort
{
public:
    explicit Undistort(const cv::Rect& rect);

    cv::Point transform_pixel(int col, int row);

    // This function interpolates radmap
    cv::Point2d transform_point(double px, double py);
    inline auto transform_point(const cv::Point2d& p)
    {
        return transform_point(p.x, p.y);
    }

    // the "real" forward transformation, which could be slow
    virtual cv::Point2d slow_transform_point(double px, double py) = 0;
    virtual cv::Point2d inverse_transform_point(double px, double py) = 0;

    virtual cv::Mat unmap(const cv::Mat& src, cv::Mat& rawimg) = 0;

    bool rectilinear_equivalent() const;

    void set_rectilinear_equivalent(bool b);

    void set_max_val(const cv::Point2d& maxv);

    void set_allow_crop(bool crop);

    void apply_padding(std::vector<cv::Mat>& images);

public:
    cv::Point2d centre;
    cv::Point2d offset;
    std::vector<double> radmap; // a vector of the transformed radius sampled at
                                // uniform spacing in the untransformed radius
    cv::Point2d max_val;

    bool rectilinear = false;
    bool allow_crop = true;

protected:
    // called from derived class constructor once parameters are known
    void build_radmap();
    cv::Mat unmap_base(const cv::Mat& src, cv::Mat& rawimg, int pad_left,
                       int pad_top);
    void estimate_padding(const cv::Mat& src, int& pad_left, int& pad_top);

protected:
    cv::Point last_padding;
};
