#include "imatest_crop.h"

#include "logger.h"

namespace {

bool robust_line(const std::vector<cv::Point2d>& points, double il_thresh,
                 cv::Point2d& best_sol)
{
    double best_score = -1e20;
    cv::Point2d sol;

    bool done = false;
    int tries = 0;
    int workable = 0;
    for (size_t i = 0; i < points.size() - 1 && !done; i++) {
        for (size_t j = i + 1; j < points.size() && !done; j++) {
            tries++;

            sol.x = (points[j].y - points[i].y) / (points[j].x - points[i].x);
            sol.y = points[i].y;

            double sum_x = 0;
            double sum_y = 0;
            double sum_xx = 0;
            double sum_xy = 0;
            double n = 0;

            for (const auto& point : points) {
                double pred = point.x * sol.x + sol.y;
                double e = std::abs(pred - point.y);
                if (e < il_thresh) {
                    n++;
                    sum_x += point.x;
                    sum_y += point.y;
                    sum_xy += point.x * point.y;
                    sum_xx += point.x * point.x;
                }
            }

            if (n > 0.5 * points.size()) {
                workable++;
                double b =
                    (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
                double a = (sum_y - b * sum_x) / n;

                n = 0;
                double rmse = 0;
                for (size_t k = 0; k < points.size(); k++) {
                    double pred = points[k].x * b + a;
                    double e = fabs(pred - points[k].y);
                    if (e < il_thresh) {
                        n++;
                        rmse += e * e;
                    }
                }
                rmse = sqrt(rmse / n);

                double score = (points.size() - n) + rmse;

                if (score > best_score) {
                    best_score = score;
                    best_sol = cv::Point2d(b, a);
                    if (n > 0.6 * points.size()) {
                        done = true;
                    }
                }
            }
        }
    }

    logger.debug("Imatest_crop: line fit after %d tries (%d workable), "
                 "score=%lg, got line %lg*x + %lg\n",
                 tries, workable, best_score, best_sol.x, best_sol.y);

    return best_score > 0;
}

bool sanity_check(const std::vector<cv::Point2d>& data, int width)
{
    if (data.size() < 10)
        return false;
    if (data.front().x > 0.3 * width)
        return false;
    if (data.back().x < 0.7 * width)
        return false;
    return true;
}

} // namespace

Imatest_cropper::Imatest_cropper(cv::Mat& in_img) : Cropper(in_img)
{
    std::vector<float> histo(8194, 0.0);
    const uint16_t* p = (uint16_t*)in_img.data;
    const uint16_t* sentinel = p + in_img.total();
    while (p < sentinel) {
        histo[*p >> 3]++;
        p += 1;
    }
    double total = 0;
    double sum = 0;
    for (size_t k = 0; k < histo.size(); k++) {
        total += histo[k];
        sum += histo[k] * k;
    }

    // compute Otsu threshold
    double sum_b = 0;
    double w_b = 0;
    double max = 0;
    double thresh1 = 0;
    double thresh2 = 0;
    for (size_t k = 0; k < histo.size(); k++) {
        w_b += histo[k];
        double w_f = total - w_b;
        if (w_f == 0) {
            break;
        }

        sum_b += k * histo[k];
        double m_b = sum_b / w_b;
        double m_f = (sum - sum_b) / w_f;
        double between = w_b * w_f * (m_b - m_f) * (m_b - m_f);
        if (between >= max) {
            thresh1 = k;
            if (between > max) {
                thresh2 = k;
            }
            max = between;
        }
    }
    uint16_t threshold = lrint((thresh1 + thresh2) * 0.5 * 8);

    logger.debug("Imatest_crop: Otsu threshold = %d\n", threshold);

    std::vector<cv::Point2d> top_points;
    std::vector<cv::Point2d> bot_points;
    const int top_row_limit = in_img.rows / 5;
    const int bot_row_limit = in_img.rows - 1 - in_img.rows / 5;
    const int bar_height_max = in_img.rows * 0.035;
    const int colskip = in_img.cols < 1024 ? 1 : 4;
    for (int col = 0; col < in_img.cols; col += colskip) {
        int row = 0;
        while (row < top_row_limit && in_img.at<uint16_t>(row, col) > threshold)
            row++;
        if (row != top_row_limit) {
            int black_start = row;
            while (row < black_start + bar_height_max &&
                   in_img.at<uint16_t>(row, col) <= threshold) {
                row++;
            }
            if (row != black_start + bar_height_max) {
                top_points.push_back(cv::Point2d(col, row));
            }
        }
        row = in_img.rows - 1;
        while (row > bot_row_limit &&
               in_img.at<uint16_t>(row, col) > threshold) {
            row--;
        }
        if (row != bot_row_limit) {
            int black_start = row;
            while (row > black_start - bar_height_max &&
                   in_img.at<uint16_t>(row, col) <= threshold) {
                row--;
            }
            if (row != black_start - bar_height_max) {
                bot_points.push_back(cv::Point2d(col, row));
            }
        }
    }

    if (!(sanity_check(top_points, in_img.cols) &&
          sanity_check(bot_points, in_img.cols))) {
        rstart_ = 0;
        height_ = in_img.rows;
        return;
    }

    cv::Point2d top_line, bot_line;
    bool line_fit_ok = robust_line(top_points, 3, top_line) &&
                       robust_line(bot_points, 3, bot_line);
    if (!line_fit_ok) {
        rstart_ = 0;
        height_ = in_img.rows;
        return;
    }

    double top_extreme =
        std::max(top_line.x * top_points.front().x + top_line.y + 1,
                 top_line.x * top_points.back().x + top_line.y + 1);

    double bot_extreme =
        std::min(bot_line.x * bot_points.front().x + bot_line.y - 1,
                 bot_line.x * bot_points.back().x + bot_line.y - 1);

    logger.debug("Imatest_crop: proposed cuts between rows %d and %d\n",
                 (int)top_extreme, (int)bot_extreme);

    rstart_ = std::max(size_t(0), (size_t)lrint(top_extreme));
    height_ =
        std::min(size_t(in_img.rows), (size_t)lrint(bot_extreme - top_extreme));

    cstart_ = 0;
    width_ = in_img.cols;

    double csum = total;
    int bidx = histo.size() - 1;
    while (bidx > 0 && csum > 0.75 * total) {
        csum -= histo[bidx--];
    }

    max_brightness = bidx * 8;
}

void Imatest_cropper::fill_bars(const cv::Mat& X)
{
    if (rstart_ > 0) {
        cv::Mat top_block(X, cv::Rect(0, 0, X.cols, rstart_));
        top_block = max_brightness;
    }

    if (height_ < X.rows - 1) {
        cv::Mat bot_block(X, cv::Rect(0, rstart_ + height_, X.cols,
                                      X.rows - 1 - (rstart_ + height_)));
        bot_block = max_brightness;
    }
}
