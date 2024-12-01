#include "cameradraw.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "srgbrender.h"

Camera_draw::Camera_draw(const cv::Mat& img,
                         const Distance_scale& distance_scale,
                         double scaling_factor, int pad)
    : img(img),
      distance_scale(distance_scale),
      scaling_factor(scaling_factor),
      psf(distance_scale.page_scale_factor)
{
    rimg = gray_to_colour(img, scaling_factor, pad);
    if (scaling_factor != 1.0) {
        this->distance_scale.img_scale *= scaling_factor;
        this->distance_scale.prin *= scaling_factor;
    }
}

void Camera_draw::arc_with_arrow(int axis, double rad, double offset,
                                 const cv::Scalar& color, bool tp)
{
    std::vector<cv::Point2d> cpts;

    double st = tp ? M_PI : 0;                         // start angle
    double et = tp ? M_PI - M_PI * 0.55 : M_PI * 0.55; // end angle
    double ainc = M_PI / 200.0 * (tp ? -1 : 1);
    double x = 0;
    double y = 0;
    double z = 0;
    for (double theta = st; (tp && theta > et) || (!tp && theta < et);
         theta += ainc) {
        switch (axis) {
            case 0: // arrow around x-axis
                x = 0;
                y = rad * cos(theta) - offset;
                z = -rad * sin(theta);
                break;
            case 1: // arrow around y-axis
                x = rad * cos(theta) - offset;
                y = 0;
                z = -rad * sin(theta);
                break;
            case 2: // arrow around z-axis
                x = rad * cos(theta) - offset;
                y = rad * sin(theta) - offset;
                z = -offset;
                break;
        }
        cpts.push_back(
            distance_scale.world_to_image(psf * x, psf * y, psf * z));
    }
    curve(cpts, cv::Scalar(20, 20, 20), 4, cv::Scalar(20, 20, 20));
    curve(cpts, color, 2, color);

    {
        const double th = 0; // angle orientation, could be 180 to flip arrow?
        const double dth = 25.0 / 180.0 * M_PI;
        const double aw = 3.0;
        double rad = 10;

        cv::Point2d dpts[3] = {
            {(rad + aw) * cos(th), (rad + aw) * sin(th)},
            {(rad - aw) * cos(th + dth), (rad - aw) * sin(th + dth)},
            {(rad - aw) * cos(th - dth), (rad - aw) * sin(th - dth)}};

        std::array<cv::Point, 3> pts;
        for (int i = 0; i < 3; i++) {
            cv::Point2d pt;
            switch (axis) {
                case 0: // x-axis
                    pt = distance_scale.world_to_image(
                        psf * dpts[i].y,
                        psf * (y - (tp ? -1 : 1) * (dpts[i].x - (rad - aw))),
                        psf * z);
                    break;
                case 1: // y-axis
                    pt = distance_scale.world_to_image(
                        psf * (x - (tp ? -1 : 1) * (dpts[i].x - (rad - aw))),
                        psf * dpts[i].y, psf * z);
                    break;
                case 2: // z-axis
                    pt = distance_scale.world_to_image(
                        psf * (x - (tp ? -1 : 1) * (dpts[i].x - (rad - aw))),
                        psf * (y - dpts[i].y), psf * z);
                    break;
            }

            pts[i].x = lrint(pt.x);
            pts[i].y = lrint(pt.y);
        }
        for (int i = 0; i < 3; i++) {
            cv::line(rimg, pts[i], pts[(i + 1) % 3], cv::Scalar(20, 20, 20), 2,
                     cv::LINE_AA);
        }

        cv::fillConvexPoly(rimg, pts, color, cv::LINE_AA);
    }
}

cv::Mat Camera_draw::gray_to_colour(const cv::Mat& ig_img, double sfactor,
                                    int pad)
{
    // TODO: we can downsize the image here ...
    cv::Mat channel;
    cv::Mat g_img;
    if (sfactor != 1.0) {
        cv::resize(ig_img, g_img, cv::Size(0, 0), sfactor, sfactor);
    }
    else {
        g_img = ig_img.clone();
    }

    cv::Mat merged = Srgb_render::linear_to_sRGB(g_img);

    if (pad > 0) {
        initial_rows = merged.rows;
        // TODO: maybe merge right column if image is too narrow
        merged.resize(merged.rows + pad);
        cv::rectangle(merged, cv::Point2d(0, g_img.rows),
                      cv::Point2d(merged.cols, merged.rows),
                      cv::Scalar::all(255), cv::FILLED);
    }

    return merged;
}

void Camera_draw::curve(const std::vector<cv::Point2d>& data,
                        const cv::Scalar& color1, double width,
                        const cv::Scalar& color2)
{
    double total_l = 0;
    for (size_t i{1}; i < data.size(); i++) {
        total_l += cv::norm(data[i] - data[i - 1]);
    }

    bool shade = color2[0] != 0 || color2[1] != 0 || color2[1] != 0;
    cv::Scalar blended_col = color1;

    cv::Rect bounds(0, 0, img.cols, img.rows + 5);

    int prevx = 0;
    int prevy = 0;
    double running_l = 0;
    for (size_t i = 0; i < data.size(); i++) {
        if (i > 1) {
            running_l += cv::norm(data[i] - data[i - 1]);
        }

        double progress = running_l / total_l;

        if (shade) {
            for (int k = 0; k < 3; k++) {
                blended_col[k] = color1[k] + (color2[k] - color1[k]) * progress;
            }
        }

        cv::Point head(prevx, prevy);
        cv::Point tail(lrint(data[i].x), lrint(data[i].y));
        bool inside = cv::clipLine(bounds, head, tail);

        if (i > 0 && inside) {
            cv::line(rimg, head, tail, blended_col, width, cv::LINE_AA);
        }

        prevx = tail.x;
        prevy = tail.y;
    }
}

void Camera_draw::chart_centre_marker()
{
    const cv::Scalar markerColor(0, 127 - 20, 255 - 20);
    const cv::Point2d czero = distance_scale.world_to_image(0, 0);

    for (int i = 0; i < 4; i++) {
        cv::Point2d wp(psf * 10 * cos(2.0 * i * M_PI / 4.0),
                       psf * 10 * sin(2.0 * i * M_PI / 4.0));
        cv::Point2d pt = distance_scale.world_to_image(wp.x, wp.y);
        cv::line(rimg, czero, pt, markerColor, 2, cv::LINE_AA);
    }
    cv::circle(rimg, czero, 10, markerColor, 2, cv::LINE_AA);

    int j = 0;
    for (double th = 0; j < 4; th += 2 * M_PI / 4.0, j++) {
        constexpr double dth = 10.0 / 180.0 * M_PI;
        constexpr double rad = 10;
        constexpr double aw = 2.0;

        cv::Point2d dpts[3] = {
            {(rad + aw) * cos(th), (rad + aw) * sin(th)},
            {(rad - aw) * cos(th + dth), (rad - aw) * sin(th + dth)},
            {(rad - aw) * cos(th - dth), (rad - aw) * sin(th - dth)}};

        std::array<cv::Point, 3> pts;
        for (int i = 0; i < 3; i++) {
            cv::Point2d pt =
                distance_scale.world_to_image(psf * dpts[i].x, psf * dpts[i].y);
            pts[i].x = lrint(pt.x);
            pts[i].y = lrint(pt.y);
        }

        cv::fillConvexPoly(rimg, pts, markerColor, cv::LINE_AA);
    }
}

void Camera_draw::camera_centre_marker(double dc_x, double dc_y)
{
    const cv::Scalar reticle_col(0, 127 - 50, 255 - 50);
    const cv::Point centre(dc_x, dc_y);

    constexpr double rad = 25;
    cv::circle(rimg, centre, rad, CV_RGB(20, 20, 20), 4, cv::LINE_AA);

    int i = 0;
    for (double th = M_PI / 2; i < 4; th += 2 * M_PI / 4.0, i++) {
        constexpr double dth = 12.5 / 180.0 * M_PI;

        const std::array<cv::Point, 3> pts{
            cv::Point{int((double)centre.x + (rad - 9) * cos(th)),
                      int((double)centre.y + (rad - 9) * sin(th))},
            {int((double)centre.x + (rad + 8) * cos(th + dth)),
             int((double)centre.y + (rad + 8) * sin(th + dth))},
            {int((double)centre.x + (rad + 8) * cos(th - dth)),
             int((double)centre.y + (rad + 8) * sin(th - dth))}};

        cv::fillConvexPoly(rimg, pts, CV_RGB(20, 20, 20), cv::LINE_AA);
    }

    cv::circle(rimg, centre, rad, reticle_col, 2, cv::LINE_AA);

    i = 0;
    for (double th = M_PI / 2; i < 4; th += 2 * M_PI / 4.0, i++) {
        const double dth = 10.0 / 180.0 * M_PI;

        const std::array<cv::Point, 3> pts{
            cv::Point{int((double)centre.x + (rad - 7) * cos(th)),
                      int((double)centre.y + (rad - 7) * sin(th))},
            {int((double)centre.x + (rad + 7) * cos(th + dth)),
             int((double)centre.y + (rad + 7) * sin(th + dth))},
            {int((double)centre.x + (rad + 7) * cos(th - dth)),
             int((double)centre.y + (rad + 7) * sin(th - dth))}};

        cv::fillConvexPoly(rimg, pts, reticle_col, cv::LINE_AA);
    }
}

void Camera_draw::fail_circle()
{
    const cv::Point2d cent(rimg.cols / 2, rimg.rows / 2);
    const double rad = std::min(rimg.rows, rimg.cols) / 2.0 - 20;
    const cv::Scalar red(30, 30, 255);
    cv::circle(rimg, cent, rad, red, 10, cv::LINE_AA);
    const cv::Point2d dir(rad * std::sqrt(0.5) - 2, rad * std::sqrt(0.5) - 2);
    cv::line(rimg, cent - dir, cent + dir, red, 10, cv::LINE_AA);
}

void Camera_draw::fail_with_message(const std::string& path,
                                    const std::string& message)
{
    fail_circle();

    constexpr int font = cv::FONT_HERSHEY_DUPLEX;

    cv::putText(rimg, message,
                cv::Point2d(50, initial_rows + (rimg.rows - initial_rows) / 2),
                font, 1, cv::Scalar::all(0), 1, cv::LINE_AA);

    cv::imwrite(path, rimg);
}

void Camera_draw::checkmark(const cv::Point2d& pos, const cv::Scalar& color)
{
    const cv::Point start(pos);

    {
        std::array<cv::Point, 4> tri;
        tri[0] = start;
        tri[1].x = tri[0].x + 5 * std::cos(45.0 / 180.0 * M_PI);
        tri[1].y = tri[0].y + 5 * std::sin(45.0 / 180.0 * M_PI);
        tri[2].x = tri[1].x + 20 * std::cos(-45.0 / 180.0 * M_PI);
        tri[2].y = tri[1].y + 20 * std::sin(-45.0 / 180.0 * M_PI);
        tri[3].x = tri[2].x + 1 * std::cos((180 + 45.0) / 180.0 * M_PI);
        tri[3].y = tri[2].y + 1 * std::sin((180 + 45.0) / 180.0 * M_PI);

        cv::fillConvexPoly(rimg, tri, color, cv::LINE_AA);
    }

    {
        std::array<cv::Point, 4> tri;
        tri[0] = start;
        tri[1].x = tri[0].x + 5 * std::cos(-45.0 / 180.0 * M_PI);
        tri[1].y = tri[0].y + 5 * std::sin(-45.0 / 180.0 * M_PI);
        tri[2].x = tri[1].x - 5 * std::cos(45.0 / 180.0 * M_PI);
        tri[2].y = tri[1].y - 5 * std::sin(45.0 / 180.0 * M_PI);
        tri[3].x = tri[2].x + 5 * std::cos((180 - 45.0) / 180.0 * M_PI);
        tri[3].y = tri[2].y + 5 * std::sin((180 - 45.0) / 180.0 * M_PI);

        cv::fillConvexPoly(rimg, tri, color, cv::LINE_AA);
    }
}

void Camera_draw::crossmark(const cv::Point2d& pos, const cv::Scalar& color)
{
    const cv::Point cent(pos);

    {
        const cv::Point2d dir(std::cos(M_PI * 0.25), std::sin(M_PI * 0.25));

        std::array<cv::Point, 4> tri;
        tri[0].x = cent.x - 10 * dir.x + 2.5 * dir.y;
        tri[0].y = cent.y - 10 * dir.y - 2.5 * dir.x;
        tri[1].x = tri[0].x - 5 * dir.y;
        tri[1].y = tri[0].y + 5 * dir.x;
        tri[2].x = tri[1].x + 20 * dir.x;
        tri[2].y = tri[1].y + 20 * dir.y;
        tri[3].x = tri[2].x + 5 * dir.y;
        tri[3].y = tri[2].y - 5 * dir.x;

        cv::fillConvexPoly(rimg, tri, color, cv::LINE_AA);
    }

    {
        const cv::Point2d dir(std::cos(M_PI * 0.75), std::sin(M_PI * 0.75));

        std::array<cv::Point, 4> tri;
        tri[0].x = cent.x - 10 * dir.x + 2.5 * dir.y;
        tri[0].y = cent.y - 10 * dir.y - 2.5 * dir.x;
        tri[1].x = tri[0].x - 5 * dir.y;
        tri[1].y = tri[0].y + 5 * dir.x;
        tri[2].x = tri[1].x + 20 * dir.x;
        tri[2].y = tri[1].y + 20 * dir.y;
        tri[3].x = tri[2].x + 5 * dir.y;
        tri[3].y = tri[2].y - 5 * dir.x;

        cv::fillConvexPoly(rimg, tri, color, cv::LINE_AA);
    }
}

void Camera_draw::alpha_block(const cv::Point2d& p, const cv::Size& s,
                              const cv::Scalar& col, double alpha)
{
    // trim to img bounds
    int e_width = std::min((int)p.x + (int)s.width, rimg.cols - 1) - p.x;
    int s_row = std::max(0, (int)p.y - (int)s.height - 1);
    int e_row = std::min((int)p.y + 2, rimg.rows - 1);
    cv::Mat roi = rimg(cv::Rect(p.x, s_row, e_width, e_row - s_row));
    cv::Mat cblock(roi.size(), CV_8UC3, col);
    cv::addWeighted(cblock, alpha, roi, 1.0 - alpha, 0.0, roi);
}

void Camera_draw::text(double x, double y, double z, const cv::Scalar& color,
                       const std::string& text)
{
    // x, y and z are in world coordinates

    constexpr int font = cv::FONT_HERSHEY_DUPLEX;

    const auto textpos = distance_scale.world_to_image(x, y, z);
    cv::putText(rimg, text, textpos, font, 1, CV_RGB(50, 50, 50), 3,
                cv::LINE_AA);
    cv::putText(rimg, text, textpos, font, 1, CV_RGB(20, 20, 20), 2,
                cv::LINE_AA);
    cv::putText(rimg, text, textpos, font, 1, color, 1, cv::LINE_AA);
}

void Camera_draw::text(const cv::Point2d& pos, const cv::Scalar& color,
                       const std::string& text)
{
    // pos is in pixel coordinates
    constexpr int font = cv::FONT_HERSHEY_DUPLEX;

    cv::putText(rimg, text, pos, font, 1, color, 1, cv::LINE_AA);
}

void Camera_draw::text_block(double x, double y, double z,
                             const cv::Scalar& color, const std::string& text)
{
    constexpr int font = cv::FONT_HERSHEY_DUPLEX;

    const auto textpos = distance_scale.world_to_image(x, y, z);

    int baseline;
    const auto ts = cv::getTextSize(text, font, 1, 3, &baseline);

    alpha_block(textpos, ts, CV_RGB(255, 255, 255), 0.5);
    cv::putText(rimg, text, textpos, font, 1, CV_RGB(50, 50, 50), 3,
                cv::LINE_AA);
    cv::putText(rimg, text, textpos, font, 1, CV_RGB(20, 20, 20), 2,
                cv::LINE_AA);
    cv::putText(rimg, text, textpos, font, 1, color, 1, cv::LINE_AA);
}

void Camera_draw::text_block_ra(double x, double y, double z,
                                const cv::Scalar& color,
                                const std::string& text)
{
    constexpr int font = cv::FONT_HERSHEY_DUPLEX;

    auto textpos = distance_scale.world_to_image(x, y, z);

    int baseline;
    const auto ts = cv::getTextSize(text, font, 1, 3, &baseline);
    textpos.x -= ts.width;

    alpha_block(textpos, ts, CV_RGB(255, 255, 255), 0.5);
    cv::putText(rimg, text, textpos, font, 1, CV_RGB(50, 50, 50), 3,
                cv::LINE_AA);
    cv::putText(rimg, text, textpos, font, 1, CV_RGB(20, 20, 20), 2,
                cv::LINE_AA);
    cv::putText(rimg, text, textpos, font, 1, color, 1, cv::LINE_AA);
}
