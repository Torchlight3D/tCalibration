#include "mtfrenderermfprofile.h"

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "focussurface.h"

Mtf_renderer_mfprofile::Mtf_renderer_mfprofile(
    Distance_scale& distance_scale, const std::string& wdir,
    const std::string& prof_fname, const cv::Mat& img,
    [[maybe_unused]] bool lpmm_mode, [[maybe_unused]] double pixel_size)
    : wdir(wdir), prname(prof_fname), img(img), distance_scale(distance_scale)
{
}

void Mtf_renderer_mfprofile::render(const std::vector<Block>&)
{
    LOG(FATAL) << "This function should not be used. Aborting";
    return;
}

void Mtf_renderer_mfprofile::render(
    const std::vector<Mtf_profile_sample>& samples)
{
    cv::Point2d centroid(0, 0);

    if (samples.size() < 10) { // probably not a valid image for profiles
        return;
    }

    if (!distance_scale.fiducials_found) {
        LOG(ERROR) << "No valid fiducials found, refusing to generate Focus "
                      "chart output.\nAre you using the right chart type?";
        return;
    }

    FILE* fout = fopen("points.txt", "wt");
    std::vector<Mtf_profile_sample> data;
    for (size_t i = 0; i < samples.size(); i++) {
        if (true) { // perform some spatial bounding here

            cv::Point2d pos = distance_scale.estimate_world_coords(
                samples[i].p.x, samples[i].p.y);

            data.push_back(Mtf_profile_sample(
                pos, samples[i].mtf, samples[i].angle, samples[i].quality));
            fprintf(fout, "%lf %lf %lf\n", pos.x, pos.y, samples[i].mtf);
        }
    }
    fclose(fout);

    Focus_surface pf(data, 3, 2, distance_scale);

    cv::Mat channel(img.rows, img.cols, CV_8UC1);
    double imin;
    double imax;
    cv::minMaxLoc(img, &imin, &imax);
    img.convertTo(channel, CV_8U, 255.0 / (imax - imin), 0);

    std::vector<cv::Mat> channels;
    channels.push_back(channel);
    channels.push_back(channel);
    channels.push_back(channel);
    cv::Mat merged;
    merge(channels, merged);
    int initial_rows = merged.rows;
    merged.resize(merged.rows + 100);

    // TODO: find a way to sample the values for display purposes?
    //    for (size_t i = 1; i < points.size(); i += 3) {
    //        Point2d d = points[i].p - zero;
    //        Point2d coord(d.x * longitudinal.x + d.y * longitudinal.y,
    //                      d.x * transverse.x + d.y * transverse.y);

    //        if (fabs(coord.x - cpx) > covxx * radxf ||
    //            fabs(coord.y - cpy) > covyy * radyf) {
    //            continue; // point was excluded
    //        }

    //        int baseline = 0;
    //        char buffer[1024];
    //        sprintf(buffer, "%03d", (int)lrint(points[i].mtf * 1000));
    //        cv::Size ts = cv::getTextSize(buffer, cv::FONT_HERSHEY_SIMPLEX,
    //        0.5, 1,
    //                                      &baseline);
    //        cv::Point to(-ts.width / 2, ts.height / 2);
    //        to.x += points[i].p.x;
    //        to.y += points[i].p.y;
    //        cv::putText(merged, buffer, to, cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //                    CV_RGB(20, 20, 20), 2.5, CV_AA);
    //        cv::putText(merged, buffer, to, cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //                    CV_RGB(50, 255, 255), 1, CV_AA);
    //    }

    draw_curve(merged, pf.ridge_peaks, cv::Scalar(30, 30, 255), 2, true);
    draw_curve(merged, pf.ridge, cv::Scalar(30, 255, 30), 3);
    draw_curve(merged, pf.ridge_p05, cv::Scalar(100, 100, 200), 1);
    draw_curve(merged, pf.ridge_p95, cv::Scalar(100, 100, 200), 1);

    rectangle(merged, cv::Point2d(0, initial_rows),
              cv::Point2d(merged.cols, merged.rows), cv::Scalar::all(255),
              cv::FILLED);

    int font = cv::FONT_HERSHEY_DUPLEX;
    char tbuffer[1024];
    sprintf(tbuffer,
            "Focus peak at depth %.1lf mm [%.1lf,%.1lf] relative to chart "
            "origin",
            pf.focus_peak, pf.focus_peak_p05, pf.focus_peak_p95);
    cv::putText(
        merged, tbuffer,
        cv::Point2d(50, initial_rows + (merged.rows - initial_rows) / 2), font,
        1, cv::Scalar::all(0), 1, cv::LINE_AA);

    cv::imwrite(wdir + prname, merged);
}

void Mtf_renderer_mfprofile::draw_curve(cv::Mat& image,
                                        const std::vector<cv::Point2d>& data,
                                        cv::Scalar col, double width,
                                        bool points)
{
    int prevx = 0;
    int prevy = 0;
    for (size_t i = 0; i < data.size(); i++) {
        cv::Point2d pos = distance_scale.world_to_image(data[i].x, data[i].y);

        int ix = lrint(pos.x);
        int iy = lrint(pos.y);
        if (ix >= 0 && ix < image.cols && iy >= 0 && iy < image.rows && i > 0) {
            if (points) {
                cv::line(image, cv::Point2d(ix, iy), cv::Point2d(ix, iy), col,
                         width, cv::LINE_AA);
            }
            else {
                cv::line(image, cv::Point2d(prevx, prevy), cv::Point2d(ix, iy),
                         col, width, cv::LINE_AA);
            }
        }

        prevx = ix;
        prevy = iy;
    }
}
