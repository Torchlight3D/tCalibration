#include "camera_draw.h"

#include <opencv2/imgcodecs.hpp>

#include "srgb_render.h"

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
                                 cv::Scalar colour, bool tp)
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
    curve(cpts, colour, 2, colour);
    {
        const double th = 0; // angle orientation, could be 180 to flip arrow?
        const double dth = 25.0 / 180.0 * M_PI;
        const double aw = 3.0;
        double rad = 10;

        cv::Point2d dpts[3] = {
            {(rad + aw) * cos(th), (rad + aw) * sin(th)},
            {(rad - aw) * cos(th + dth), (rad - aw) * sin(th + dth)},
            {(rad - aw) * cos(th - dth), (rad - aw) * sin(th - dth)}};

        cv::Point pts[3];
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
        cv::fillConvexPoly(rimg, (const cv::Point*)&pts, 3, colour,
                           cv::LINE_AA);
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
        rectangle(merged, cv::Point2d(0, g_img.rows),
                  cv::Point2d(merged.cols, merged.rows), cv::Scalar::all(255),
                  cv::FILLED);
    }

    return merged;
}

void Camera_draw::curve(const std::vector<cv::Point2d>& data, cv::Scalar col,
                        double width, cv::Scalar col2)
{
    int prevx = 0;
    int prevy = 0;

    double total_l = 0;
    for (size_t i = 1; i < data.size(); i++) {
        double dx = data[i].x - data[i - 1].x;
        double dy = data[i].y - data[i - 1].y;
        total_l += sqrt(dx * dx + dy * dy);
    }

    bool shade = col2[0] != 0 || col2[1] != 0 || col2[1] != 0;
    cv::Scalar blended_col = col;

    cv::Rect bounds(0, 0, img.cols, img.rows + 5);

    double running_l = 0;
    for (size_t i = 0; i < data.size(); i++) {
        if (i > 1) {
            double dx = data[i].x - data[i - 1].x;
            double dy = data[i].y - data[i - 1].y;
            running_l += sqrt(dx * dx + dy * dy);
        }

        double progress = running_l / total_l;

        if (shade) {
            for (int k = 0; k < 3; k++) {
                blended_col[k] = col[k] + (col2[k] - col[k]) * progress;
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
    cv::Scalar mark_col(0, 127 - 20, 255 - 20);
    cv::Point2d czero = distance_scale.world_to_image(0, 0);

    for (int i = 0; i < 4; i++) {
        cv::Point2d wp(psf * 10 * cos(2.0 * i * M_PI / 4.0),
                       psf * 10 * sin(2.0 * i * M_PI / 4.0));
        cv::Point2d pt = distance_scale.world_to_image(wp.x, wp.y);
        cv::line(rimg, czero, pt, mark_col, 2, cv::LINE_AA);
    }
    cv::circle(rimg, czero, 10, mark_col, 2, cv::LINE_AA);

    int j = 0;
    for (double th = 0; j < 4; th += 2 * M_PI / 4.0, j++) {
        const double dth = 10.0 / 180.0 * M_PI;
        double rad = 10;
        const double aw = 2.0;

        cv::Point2d dpts[3] = {
            {(rad + aw) * cos(th), (rad + aw) * sin(th)},
            {(rad - aw) * cos(th + dth), (rad - aw) * sin(th + dth)},
            {(rad - aw) * cos(th - dth), (rad - aw) * sin(th - dth)}};

        cv::Point pts[3];
        for (int i = 0; i < 3; i++) {
            cv::Point2d pt =
                distance_scale.world_to_image(psf * dpts[i].x, psf * dpts[i].y);
            pts[i].x = lrint(pt.x);
            pts[i].y = lrint(pt.y);
        }

        cv::fillConvexPoly(rimg, (const cv::Point*)&pts, 3, mark_col,
                           cv::LINE_AA);
    }
}

void Camera_draw::camera_centre_marker(double dc_x, double dc_y)
{
    cv::Scalar reticle_col(0, 127 - 50, 255 - 50);
    cv::Point centre(dc_x, dc_y);

    const double rad = 25;
    cv::circle(rimg, centre, rad, cv::Scalar(20, 20, 20), 4, cv::LINE_AA);
    int i = 0;
    for (double th = M_PI / 2; i < 4; th += 2 * M_PI / 4.0, i++) {
        const double dth = 12.5 / 180.0 * M_PI;

        cv::Point pts[3] = {
            {int((double)centre.x + (rad - 9) * cos(th)),
             int((double)centre.y + (rad - 9) * sin(th))},
            {int((double)centre.x + (rad + 8) * cos(th + dth)),
             int((double)centre.y + (rad + 8) * sin(th + dth))},
            {int((double)centre.x + (rad + 8) * cos(th - dth)),
             int((double)centre.y + (rad + 8) * sin(th - dth))}};

        cv::fillConvexPoly(rimg, (const cv::Point*)&pts, 3,
                           cv::Scalar(20, 20, 20), cv::LINE_AA);
    }
    cv::circle(rimg, centre, rad, reticle_col, 2, cv::LINE_AA);
    i = 0;
    for (double th = M_PI / 2; i < 4; th += 2 * M_PI / 4.0, i++) {
        const double dth = 10.0 / 180.0 * M_PI;

        cv::Point pts[3] = {
            {int((double)centre.x + (rad - 7) * cos(th)),
             int((double)centre.y + (rad - 7) * sin(th))},
            {int((double)centre.x + (rad + 7) * cos(th + dth)),
             int((double)centre.y + (rad + 7) * sin(th + dth))},
            {int((double)centre.x + (rad + 7) * cos(th - dth)),
             int((double)centre.y + (rad + 7) * sin(th - dth))}};

        cv::fillConvexPoly(rimg, (const cv::Point*)&pts, 3, reticle_col,
                           cv::LINE_AA);
    }
}

void Camera_draw::fail_circle()
{
    cv::Point2d cent(rimg.cols / 2, rimg.rows / 2);
    double rad = std::min(rimg.rows, rimg.cols) / 2.0 - 20;
    cv::Scalar red(30, 30, 255);

    cv::circle(rimg, cent, rad, red, 10, cv::LINE_AA);
    cv::Point2d dir(rad * sqrt(0.5) - 2, rad * sqrt(0.5) - 2);
    cv::line(rimg, cent - dir, cent + dir, red, 10, cv::LINE_AA);
}

void Camera_draw::fail_with_message(const std::string& path,
                                    const std::string& s)
{
    fail_circle();

    char tbuffer[1024];
    int font = cv::FONT_HERSHEY_DUPLEX;
    sprintf(tbuffer, "%s", s.c_str());
    cv::putText(rimg, tbuffer,
                cv::Point2d(50, initial_rows + (rimg.rows - initial_rows) / 2),
                font, 1, cv::Scalar::all(0), 1, cv::LINE_AA);

    cv::imwrite(path, rimg);
}

void Camera_draw::checkmark(const cv::Point2d& pos, const cv::Scalar& colour)
{
    cv::Point tri[5];

    cv::Point start(pos.x, pos.y);

    tri[0].x = start.x;
    tri[0].y = start.y;
    tri[1].x = tri[0].x + 5 * cos(45.0 / 180.0 * M_PI);
    tri[1].y = tri[0].y + 5 * sin(45.0 / 180.0 * M_PI);
    tri[2].x = tri[1].x + 20 * cos(-45.0 / 180.0 * M_PI);
    tri[2].y = tri[1].y + 20 * sin(-45.0 / 180.0 * M_PI);
    tri[3].x = tri[2].x + 1 * cos((180 + 45.0) / 180.0 * M_PI);
    tri[3].y = tri[2].y + 1 * sin((180 + 45.0) / 180.0 * M_PI);

    cv::fillConvexPoly(rimg, (const cv::Point*)&tri, 4, colour, cv::LINE_AA);

    tri[0].x = start.x;
    tri[0].y = start.y;
    tri[1].x = tri[0].x + 5 * cos(-45.0 / 180.0 * M_PI);
    tri[1].y = tri[0].y + 5 * sin(-45.0 / 180.0 * M_PI);
    tri[2].x = tri[1].x - 5 * cos(45.0 / 180.0 * M_PI);
    tri[2].y = tri[1].y - 5 * sin(45.0 / 180.0 * M_PI);
    tri[3].x = tri[2].x + 5 * cos((180 - 45.0) / 180.0 * M_PI);
    tri[3].y = tri[2].y + 5 * sin((180 - 45.0) / 180.0 * M_PI);

    cv::fillConvexPoly(rimg, (const cv::Point*)&tri, 4, colour, cv::LINE_AA);
}

void Camera_draw::crossmark(const cv::Point2d& pos, const cv::Scalar& colour)
{
    cv::Point tri[5];

    cv::Point cent(pos.x, pos.y);

    cv::Point2d dir(cos(M_PI / 4), sin(M_PI / 4));
    tri[0].x = cent.x - 10 * dir.x + 2.5 * dir.y;
    tri[0].y = cent.y - 10 * dir.y - 2.5 * dir.x;
    tri[1].x = tri[0].x - 5 * dir.y;
    tri[1].y = tri[0].y + 5 * dir.x;
    tri[2].x = tri[1].x + 20 * dir.x;
    tri[2].y = tri[1].y + 20 * dir.y;
    tri[3].x = tri[2].x + 5 * dir.y;
    tri[3].y = tri[2].y - 5 * dir.x;

    cv::fillConvexPoly(rimg, (const cv::Point*)&tri, 4, colour, cv::LINE_AA);

    dir = cv::Point2d(cos(M_PI / 4 + M_PI / 2), sin(M_PI / 4 + M_PI / 2));
    tri[0].x = cent.x - 10 * dir.x + 2.5 * dir.y;
    tri[0].y = cent.y - 10 * dir.y - 2.5 * dir.x;
    tri[1].x = tri[0].x - 5 * dir.y;
    tri[1].y = tri[0].y + 5 * dir.x;
    tri[2].x = tri[1].x + 20 * dir.x;
    tri[2].y = tri[1].y + 20 * dir.y;
    tri[3].x = tri[2].x + 5 * dir.y;
    tri[3].y = tri[2].y - 5 * dir.x;

    cv::fillConvexPoly(rimg, (const cv::Point*)&tri, 4, colour, cv::LINE_AA);
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
