#pragma once

#include <opencv2/imgproc.hpp>

#include "distance_scale.h"

class Camera_draw
{
public:
    Camera_draw(const cv::Mat& img, const Distance_scale& distance_scale,
                double scaling_factor = 1.0, int pad = 0);

    void arc_with_arrow(int axis, double rad, double offset, cv::Scalar colour,
                        bool tp = false);

    cv::Mat gray_to_colour(const cv::Mat& ig_img, double sfactor, int pad = 0);

    void curve(const std::vector<cv::Point2d>& data, cv::Scalar col,
               double width, cv::Scalar col2 = {});

    void chart_centre_marker();

    void camera_centre_marker(double dc_x, double dc_y);
    void fail_circle();

    void fail_with_message(const std::string& path, const std::string& s);

    void checkmark(const cv::Point2d& pos, const cv::Scalar& colour);

    void crossmark(const cv::Point2d& pos, const cv::Scalar& colour);

    void alpha_block(const cv::Point2d& p, const cv::Size& s,
                     const cv::Scalar& col, double alpha);

    template <class... T>
    void text(double x, double y, double z, cv::Scalar& colour, T... t)
    { // x, y and z are in world coordinates

        const int font = cv::FONT_HERSHEY_DUPLEX;
        char tbuffer[4096];

        sprintf(tbuffer, t..., 0);
        cv::Point2d textpos = distance_scale.world_to_image(x, y, z);
        cv::putText(rimg, tbuffer, textpos, font, 1, CV_RGB(50, 50, 50), 3,
                    cv::LINE_AA);
        cv::putText(rimg, tbuffer, textpos, font, 1, CV_RGB(20, 20, 20), 2,
                    cv::LINE_AA);
        cv::putText(rimg, tbuffer, textpos, font, 1, colour, 1, cv::LINE_AA);
    }

    template <class... T>
    void text(cv::Point2d pos, cv::Scalar& colour, T... t)
    { // pos is in pixel coordinates

        const int font = cv::FONT_HERSHEY_DUPLEX;
        char tbuffer[4096];

        sprintf(tbuffer, t...);
        cv::putText(rimg, tbuffer, pos, font, 1, colour, 1, cv::LINE_AA);
    }

    template <class... T>
    void text_block(double x, double y, double z, cv::Scalar& colour, T... t)
    {
        const int font = cv::FONT_HERSHEY_DUPLEX;
        int baseline;
        cv::Size ts;
        char tbuffer[4096];

        sprintf(tbuffer, t...);
        cv::Point2d textpos = distance_scale.world_to_image(x, y, z);
        ts = cv::getTextSize(tbuffer, font, 1, 3, &baseline);
        alpha_block(textpos, ts, CV_RGB(255, 255, 255), 0.5);
        cv::putText(rimg, tbuffer, textpos, font, 1, CV_RGB(50, 50, 50), 3,
                    cv::LINE_AA);
        cv::putText(rimg, tbuffer, textpos, font, 1, CV_RGB(20, 20, 20), 2,
                    cv::LINE_AA);
        cv::putText(rimg, tbuffer, textpos, font, 1, colour, 1, cv::LINE_AA);
    }

    template <class... T>
    void text_block_ra(double x, double y, double z, cv::Scalar& colour, T... t)
    {
        const int font = cv::FONT_HERSHEY_DUPLEX;
        int baseline;
        cv::Size ts;
        char tbuffer[4096];

        sprintf(tbuffer, t...);
        cv::Point2d textpos = distance_scale.world_to_image(x, y, z);
        ts = cv::getTextSize(tbuffer, font, 1, 3, &baseline);
        textpos.x -= ts.width;
        alpha_block(textpos, ts, CV_RGB(255, 255, 255), 0.5);
        cv::putText(rimg, tbuffer, textpos, font, 1, CV_RGB(50, 50, 50), 3,
                    cv::LINE_AA);
        cv::putText(rimg, tbuffer, textpos, font, 1, CV_RGB(20, 20, 20), 2,
                    cv::LINE_AA);
        cv::putText(rimg, tbuffer, textpos, font, 1, colour, 1, cv::LINE_AA);
    }

public:
    const cv::Mat img;
    Distance_scale distance_scale;
    double scaling_factor;

    cv::Mat rimg;     // rendered output image
    double psf = 1.0; // page scale factor

    int initial_rows;
};
