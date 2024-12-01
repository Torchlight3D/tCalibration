#pragma once

#include "distancescale.h"

class Camera_draw
{
public:
    Camera_draw(const cv::Mat& img, const Distance_scale& distance_scale,
                double scaling_factor = 1.0, int pad = 0);

    void arc_with_arrow(int axis, double rad, double offset,
                        const cv::Scalar& color, bool tp = false);

    cv::Mat gray_to_colour(const cv::Mat& ig_img, double sfactor, int pad = 0);

    void curve(const std::vector<cv::Point2d>& data, const cv::Scalar& color1,
               double width, const cv::Scalar& color2 = {});

    void chart_centre_marker();

    void camera_centre_marker(double dc_x, double dc_y);
    void fail_circle();

    void fail_with_message(const std::string& path, const std::string& s);

    void checkmark(const cv::Point2d& pos, const cv::Scalar& color);

    void crossmark(const cv::Point2d& pos, const cv::Scalar& colour);

    void alpha_block(const cv::Point2d& p, const cv::Size& s,
                     const cv::Scalar& col, double alpha);

    void text(double x, double y, double z, const cv::Scalar& color,
              const std::string& text);

    void text(const cv::Point2d& pos, const cv::Scalar& color,
              const std::string& text);

    void text_block(double x, double y, double z, const cv::Scalar& color,
                    const std::string& text);

    void text_block_ra(double x, double y, double z, const cv::Scalar& color,
                       const std::string& text);

public:
    const cv::Mat img;
    Distance_scale distance_scale;
    double scaling_factor;

    cv::Mat rimg;     // rendered output image
    double psf = 1.0; // page scale factor

    int initial_rows;
};
