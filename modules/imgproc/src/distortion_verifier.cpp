#include "distortion_verifier.h"

#include <iostream>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "lsd.h"

namespace thoht {

using Edge = std::vector<cv::Point>;

namespace {

// The regression line: alpha * x + beta * y - gamma = 0
bool get_line_param(const Edge &edge_points, double &alpha, double &beta,
                    double &gama)
{
    double Ax{0.}, Ay{0.};
    for (const auto &point : edge_points) {
        Ax += point.x;
        Ay += point.y;
    }

    const auto point_num = edge_points.size();
    Ax /= point_num;
    Ay /= point_num;

    double Vxx{0.}, Vxy{0.}, Vyy{0.};
    for (const auto &point : edge_points) {
        double x = point.x;
        double y = point.y;
        Vxx += (x - Ax) * (x - Ax);
        Vxy += (x - Ax) * (y - Ay);
        Vyy += (y - Ay) * (y - Ay);
    }

    Vxx /= point_num;
    Vxy /= point_num;
    Vyy /= point_num;

    double theta = atan2(2 * Vxy, Vxx - Vyy) / 2.0;

    alpha = sin(theta);
    beta = -cos(theta);
    gama = Ax * sin(theta) - Ay * cos(theta);

    return true;
}

bool calculateError(
    // const std::vector<Vector4f> &lines,
    // const std::vector< std::vector<Point2i> > &edge_points,
    const std::vector<Edge> &subsampled_edges, double &d, double &d_max,
    double &d_c_median)
{
    int real_line_num = 0;
    int total_point_num = 0;
    double S{0.};
    double S_max_total{0.};
    double S_d_max{0.};

    for (const auto &edge : subsampled_edges) {
        if (edge.empty()) {
            continue;
        }

        int point_num = edge.size();
        real_line_num++;
        total_point_num += point_num;

        double alpha, beta, gama;
        get_line_param(edge, alpha, beta, gama);

        double Si_max = -10000;
        double Si_min = 10000;
        for (const auto &pt : edge) {
            double x = pt.x;
            double y = pt.y;
            double Si = (alpha * x + beta * y - gama);
            double Si_square = Si * Si;
            S += Si_square;
            Si_max = std::max(Si_max, Si);
            Si_min = std::min(Si_min, Si);
        }

        S_max_total += (Si_max - Si_min) * (Si_max - Si_min);
        double max_current = Si_max - Si_min;
        S_d_max = std::max(max_current, S_d_max);
    }

    d = std::sqrt(S / total_point_num);
    d_max = std::sqrt(S_max_total / real_line_num);
    d_c_median = S_d_max;

    LOG(INFO) << "Detected " << real_line_num << " lines. " << total_point_num
              << " edge points.";
    return true;
}

bool interpolateEdgePoints(const std::vector<cv::Vec4f> &lines,
                           const std::vector<Edge> &edge_points,
                           std::vector<Edge> &interpolated_edge_points)
{
    std::vector<Edge> sorted_edges(edge_points);
    int line_num = lines.size();

    // tan()
    double rough_angle =
        std::abs(static_cast<double>(lines[0][0] - lines[0][2]) /
                 static_cast<double>(lines[0][1] - lines[0][3]));

    // Sort line points. tan(45 deg)= 1
    if (rough_angle < 1.) {
        for (auto &edge : sorted_edges) {
            std::sort(
                edge.begin(), edge.end(),
                [](const auto &pt1, const auto &pt2) { return pt1.x < pt2.x; });
        }
    }
    else {
        for (auto &edge : sorted_edges) {
            std::sort(
                edge.begin(), edge.end(),
                [](const auto &pt1, const auto &pt2) { return pt1.y < pt2.y; });
        }
    }

    // interpolate line points
    std::vector<std::vector<double>> line_point_distance;
    line_point_distance.reserve(line_num);
    for (const auto &edge : sorted_edges) {
        if (edge.empty()) {
            continue;
        }

        std::vector<double> distances;
        distances.reserve(edge.size());
        distances.push_back(0.);
        for (int idx = 0; idx < edge.size() - 1; idx++) {
            double dist = cv::norm(edge[idx] - edge[idx + 1]);
            distances.push_back(distances[idx] + dist);
        }

        line_point_distance.push_back(distances);
    }

    interpolated_edge_points.resize(line_num);
    for (int i = 0; i < line_num; i++) {
        if (sorted_edges[i].empty())
            continue;

        double L = line_point_distance[i][line_point_distance[i].size() - 1];
        double N = sorted_edges[i].size();
        double d = L / N;
        double L_pos = 0;
        double p_pos = 1;

        interpolated_edge_points[i].push_back(sorted_edges[i][0]);

        while (p_pos < N) {
            L_pos += d;
            while (p_pos < N && L_pos > line_point_distance[i][p_pos]) {
                p_pos++;
            }

            if (p_pos >= N)
                break;

            double a = L_pos - line_point_distance[i][p_pos - 1];
            double b = line_point_distance[i][p_pos] - L_pos;
            double x1 = sorted_edges[i][p_pos - 1].x;
            double y1 = sorted_edges[i][p_pos - 1].y;
            double x2 = sorted_edges[i][p_pos].x;
            double y2 = sorted_edges[i][p_pos].y;
            double new_x = b / (a + b) * x1 + a / (a + b) * x2;
            double new_y = b / (a + b) * y1 + a / (a + b) * y2;
            // interpolated_edge_points[i].push_back(Point2i(int(new_x + 0.5),
            // int(new_y + 0.5)));
            interpolated_edge_points[i].push_back(
                cv::Point(int(new_x), int(new_y)));
        }
    }

    return true;
}

bool gaussionSubsamplePoints(const std::vector<Edge> &interpolated_edge_points,
                             std::vector<Edge> &subsampled_edge_points, int t)
{
    int line_num = interpolated_edge_points.size();
    subsampled_edge_points.reserve(line_num);

    // Generate Gauss matrix
    const double sigma = 0.8 * std::sqrt(t * t - 1);
    const double deno = 1.0 / (sigma * std::sqrt(2.0 * M_PI));
    const double nume = -1.0 / (2.0 * sigma * sigma);

    std::vector<double> gauss_matrix;
    double gauss_matrix_sum{0.};
    for (int x = -t; x <= t; x++) {
        double g = deno * exp(nume * x * x);
        gauss_matrix.push_back(g);
        gauss_matrix_sum += g;
    }

    // Normalize
    for (auto &elem : gauss_matrix) {
        elem /= gauss_matrix_sum;
    }

    // Gaussian blur
    for (const auto &edge : interpolated_edge_points) {
        int point_size = edge.size();
        if (point_size == 0)
            continue;

        Edge new_edge;
        new_edge.reserve(point_size / t + 1);
        for (int p = t; p < point_size - t; p += t) {
            double gauss_x_sum = 0;
            double gauss_y_sum = 0;
            double gauss_sum = 0;
            for (int i = -t; i <= t; i++) {
                int k = p + i;
                if (k >= 0 && k < point_size) {
                    double x = edge[k].x;
                    double y = edge[k].y;
                    gauss_x_sum += x * gauss_matrix[i + t];
                    gauss_y_sum += y * gauss_matrix[i + t];
                    gauss_sum += gauss_matrix[i + t];
                }
            }

            if (gauss_sum == 0)
                continue;

            int selected_x = static_cast<int>(gauss_x_sum / gauss_sum + 0.5);
            int selected_y = static_cast<int>(gauss_y_sum / gauss_sum + 0.5);

            new_edge.emplace_back(selected_x, selected_y);
        }

        subsampled_edge_points.push_back(new_edge);
    }

    return true;
}

} // namespace

bool DistortionVerifier::measure(const cv::Mat &src, double &_d, double &_d_max)
{
    // get line edge points
    // get line support region
    LineSegmentDetector lsd;
    std::vector<cv::Vec4f> lines;
    std::vector<Edge> line_points;
    std::vector<std::vector<int>> line_point_map;
    std::vector<double> width;
    std::vector<double> prec;
    std::vector<double> nfas;
    lsd.detect(src, lines, line_points, line_point_map, width, prec, nfas);

    // Canny edge detect
    cv::Mat blur_image;
    cv::Mat edge_image;
    cv::GaussianBlur(src, blur_image, cv::Size{3, 3}, 0.);
    cv::Canny(blur_image, edge_image, 20., 40.);

    // Get edge line point in line support region
    int line_num = lines.size();
    std::vector<Edge> edge_line_points(line_num);
    cv::Mat edge_point_image{edge_image.size(), edge_image.type(),
                             cv::Scalar(0)};
    for (int i = 0; i < line_point_map.size(); i++) {
        for (int j = 0; j < line_point_map[0].size(); j++) {
            if (edge_image.at<uchar>(i, j) == 255 &&
                line_point_map[i][j] != 0) {
                edge_point_image.at<uchar>(i, j) = 255;
                edge_line_points[line_point_map[i][j] - 1].push_back(
                    cv::Point(j, i));
            }
        }
    }

    // Visualization
    src.copyTo(_result);
    lsd.drawSegments(_result, edge_line_points);

    std::vector<Edge> interpolated_edge_points;
    interpolateEdgePoints(lines, edge_line_points, interpolated_edge_points);

    std::vector<Edge> subsampled_edge_points;
    gaussionSubsamplePoints(interpolated_edge_points, subsampled_edge_points,
                            20);

    src.copyTo(_subsample_result);
    lsd.drawSegments(_subsample_result, subsampled_edge_points);

    // Calculate error
    double d, d_max, d_c_median;
    calculateError(subsampled_edge_points, d, d_max, d_c_median);

    _d = d;
    _d_max = d_max;

    return true;
}

const cv::Mat &DistortionVerifier::lineResult() const { return _result; }

const cv::Mat &DistortionVerifier::subsamplesResult() const
{
    return _subsample_result;
}

} // namespace thoht
