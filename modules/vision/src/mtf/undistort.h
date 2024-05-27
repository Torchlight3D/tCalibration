#pragma once

#include <opencv2/core/mat.hpp>

class Undistort
{
public:
    explicit Undistort(const cv::Rect& rect);
    virtual ~Undistort() = default;

    /// Properties
    void setRectilinearEquivalent(bool on);
    bool rectilinearEquivalent() const;

    void setMaxVal(const cv::Point2d& maxv);
    const cv::Point2d& maxVal() const;

    void setAllowCrop(bool crop);
    bool allowCrop() const;

    // the "real" forward transformation, which could be slow
    virtual cv::Point2d slow_transform_point(
        const cv::Point2d& point) const = 0;
    virtual cv::Point2d inverse_transform_point(
        const cv::Point2d& point) const = 0;

    virtual cv::Mat unmap(const cv::Mat& src, cv::Mat& rawimg) = 0;

    // This function interpolates radmap
    cv::Point2d transform_point(const cv::Point2d& p);
    inline auto transform_point(double px, double py)
    {
        return transform_point(cv::Point2d{px, py});
    }
    inline auto transform_pixel(int col, int row)
    {
        return transform_point(cv::Point{col, row});
    }

    void apply_padding(std::vector<cv::Mat>& images) const;

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
