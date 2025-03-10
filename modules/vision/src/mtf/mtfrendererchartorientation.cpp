#include "mtfrendererchartorientation.h"

#include <format>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

Mtf_renderer_chart_orientation::Mtf_renderer_chart_orientation(
    const std::string& img_filename, const std::string& wdir,
    const std::string& co_fname, [[maybe_unused]] const cv::Mat& img,
    [[maybe_unused]] int gnuplot_width, Distance_scale& distance_scale,
    cv::Rect* dimension_correction)
    : Mtf_renderer(img_filename),
      wdir(wdir),
      co_fname(co_fname),
      dimension_correction(dimension_correction),
      draw(img, distance_scale, (gnuplot_width / double(img.cols)), 50)
{
}

void Mtf_renderer_chart_orientation::render(const std::vector<Block>&)
{
    Distance_scale& distance_scale = draw.distance_scale;

    if (!distance_scale.fiducials_found) {
        draw.fail_with_message(
            wdir + '/' + co_fname,
            std::string("Fiducials not found, probably incorrect chart type."));
        return;
    }

    draw.chart_centre_marker();

    // draw centre-of-camera marker
    draw.camera_centre_marker(
        draw.scaling_factor *
            (dimension_correction->width / 2 - dimension_correction->x),
        draw.scaling_factor *
            (dimension_correction->height / 2 - dimension_correction->y));

    std::vector<cv::Point2d> curve;

    cv::Scalar c_dark(20, 20, 20);
    cv::Scalar c_red(30, 30, 255);
    cv::Scalar c_green(30, 255, 30);
    cv::Scalar c_blue(255, 30, 30);

    cv::Scalar c_lred(80, 80, 255);
    cv::Scalar c_lgreen(130, 255, 130);
    cv::Scalar c_lblue(255, 182, 0);

    curve.clear();
    const double border = 35;
    double ymax = 0;
    for (; ymax > -100; ymax -= 2) {
        cv::Point2d p =
            distance_scale.world_to_image(-draw.psf, ymax * draw.psf, 0);
        if (p.x < border || p.y < border || p.x > draw.rimg.cols - 1 - border ||
            p.y > draw.rimg.rows - 1 - border)
            break;
    }

    double xmax = 0;
    for (; xmax > -100; xmax -= 2) {
        cv::Point2d p =
            distance_scale.world_to_image(xmax * draw.psf, ymax * draw.psf, 0);
        if (p.x < border || p.y < border || p.x > draw.rimg.cols - 1 - border ||
            p.y > draw.rimg.rows - 1 - border)
            break;
    }

    double zmax = 0;
    for (; zmax > -100; zmax -= 2) {
        cv::Point2d p = distance_scale.world_to_image(
            xmax * draw.psf, ymax * draw.psf, zmax * draw.psf);
        if (p.x < border || p.y < border || p.x > draw.rimg.cols - 1 - border ||
            p.y > draw.rimg.rows - 1 - border)
            break;
    }

    for (double ystep = ymax; ystep <= 0; ystep += 2) {
        curve.push_back(
            distance_scale.world_to_image(xmax * draw.psf, ystep * draw.psf));
    }
    draw.curve(curve, c_dark, 3, c_dark);
    draw.curve(curve, c_green, 2, c_green);

    // in landscape orientation, dy > dy on the y axis
    bool landscape = std::abs(curve.front().y - curve.back().y) >
                     std::abs(curve.front().x - curve.back().x);
    bool flipped = false;
    if (landscape) {
        flipped = curve.front().y < curve.back().y;
    }
    else {
        flipped = curve.front().x > curve.back().x;
    }

    curve.clear();
    for (double xstep = xmax; xstep <= 0; xstep += 2) {
        curve.push_back(
            distance_scale.world_to_image(xstep * draw.psf, ymax * draw.psf));
    }
    draw.curve(curve, c_dark, 3, c_dark);
    draw.curve(curve, c_red, 2, c_red);

    curve.clear();

    curve.push_back(
        distance_scale.world_to_image(xmax * draw.psf, ymax * draw.psf));
    curve.push_back(distance_scale.world_to_image(
        xmax * draw.psf, ymax * draw.psf, zmax * draw.psf));
    draw.curve(curve, c_dark, 3, c_dark);
    draw.curve(curve, c_lblue, 2, c_lblue);

    // if roll angle is close to 90 degrees, then assume we are in portrait
    // mode and reduce angles appropriately

    double e_roll = distance_scale.roll_angle / M_PI * 180;
    double e_pitch = distance_scale.pitch_angle / M_PI * 180;
    double e_yaw = distance_scale.yaw_angle / M_PI * 180;
    if (std::abs(e_roll - 90) < std::abs(e_roll)) {
        e_roll -= 90;
        e_pitch = distance_scale.yaw_angle / M_PI * 180;
        e_yaw = -distance_scale.pitch_angle / M_PI * 180;
    }
    else {
        if (std::abs(e_roll + 90) < std::abs(e_roll)) {
            e_roll += 90;
            e_pitch = distance_scale.yaw_angle / M_PI * 180;
            e_yaw = -distance_scale.pitch_angle / M_PI * 180;
        }
    }

    draw.arc_with_arrow(0, 10, std::abs(xmax), c_lred, (e_pitch < 0) ^ flipped);
    draw.arc_with_arrow(1, 10, std::abs(ymax), c_lgreen, (e_yaw < 0) ^ flipped);
    draw.arc_with_arrow(2, 10, std::abs(zmax), c_lblue, (e_roll < 0) ^ flipped);

    if (flipped) {
        if (landscape) {
            draw.text_block_ra(xmax * draw.psf, ymax * draw.psf,
                               zmax * draw.psf, c_lblue,
                               std::format("Roll=%.2lf", std::abs(e_roll)));
            draw.text_block_ra(xmax * draw.psf, 15 * draw.psf, 0, c_lgreen,
                               std::format("Yaw=%.2lf", std::abs(e_yaw)));
            draw.text_block(10 * draw.psf, (ymax + 20) * draw.psf, 0, c_lred,
                            std::format("Pitch=%.2lf", std::abs(e_pitch)));
        }
        else {
            draw.text_block_ra(xmax * draw.psf, (ymax + 20) * draw.psf,
                               zmax * draw.psf, c_lblue,
                               std::format("Roll=%.2lf", std::abs(e_roll)));
            draw.text_block((xmax + 15) * draw.psf, 10 * draw.psf, 0, c_lgreen,
                            std::format("Yaw=%.2lf", std::abs(e_yaw)));
            draw.text_block_ra(10 * draw.psf, ymax * draw.psf, 0, c_lred,
                               std::format("Pitch=%.2lf", std::abs(e_pitch)));
        }
    }
    else {
        if (landscape) {
            draw.text_block((xmax + 20) * draw.psf, ymax * draw.psf,
                            zmax * draw.psf, c_lblue,
                            std::format("Roll=%.2lf", std::abs(e_roll)));
            draw.text_block(xmax * draw.psf, 10 * draw.psf, 0, c_lgreen,
                            std::format("Yaw=%.2lf", std::abs(e_yaw)));
            draw.text_block_ra(10 * draw.psf, (ymax + 20) * draw.psf, 0, c_lred,
                               std::format("Pitch=%.2lf", std::abs(e_pitch)));
        }
        else {
            draw.text_block(xmax * draw.psf, (ymax + 20) * draw.psf,
                            zmax * draw.psf, c_lblue,
                            std::format("Roll=%.2lf", std::abs(e_roll)));
            draw.text_block_ra((xmax + 20) * draw.psf, 10 * draw.psf, 0,
                               c_lgreen,
                               std::format("Yaw=%.2lf", std::abs(e_yaw)));
            draw.text_block(10 * draw.psf, ymax * draw.psf, 0, c_lred,
                            std::format("Pitch=%.2lf", std::abs(e_pitch)));
        }
    }

    cv::Scalar black(0, 0, 0);

    // ugly, but we have to obtain the text height somehow
    std::string tbuffer{"dummy"};
    int baseline;
    constexpr int font = cv::FONT_HERSHEY_DUPLEX;
    cv::Size ts = cv::getTextSize(tbuffer, font, 1, 1, &baseline);

    if (distance_scale.user_provided_focal_ratio > 0) {
        draw.checkmark(
            cv::Point2d(25, draw.initial_rows + ts.height * 1 * 1.75), c_green);
        draw.text(cv::Point2d(50, draw.initial_rows + ts.height * 1 * 1.75),
                  black,
                  std::format("EXIF or user-provided focal ratio: %.2lf",
                              distance_scale.user_provided_focal_ratio));
    }
    else {
        draw.crossmark(
            cv::Point2d(35, draw.initial_rows + ts.height * 0.75 * 1.75),
            c_red);
        draw.text(cv::Point2d(50, draw.initial_rows + ts.height * 1 * 1.75),
                  black,
                  std::format("Estimated focal ratio: %.2lf",
                              distance_scale.focal_length));
    }

    cv::imwrite(wdir + '/' + co_fname, draw.rimg);
}
