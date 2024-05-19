#pragma once

#include <numbers>

#include <opencv2/core/mat.hpp>

namespace camodocal {

inline constexpr auto pi = std::numbers::pi;

template <class T>
inline constexpr T square(T x)
{
    return x * x;
}

template <class T>
inline constexpr T cube(T x)
{
    return x * x * x;
}

template <typename T>
inline double hypot3(T x, T y, T z)
{
    return std::sqrt(square(x) + square(y) + square(z));
}

template <class T>
T normalizeTheta(T theta)
{
    T normTheta = theta;

    while (normTheta < -pi) {
        normTheta += 2.0 * pi;
    }
    while (normTheta > pi) {
        normTheta -= 2.0 * pi;
    }

    return normTheta;
}

double sinc(double theta);

template <class T>
const T random(const T &a, const T &b)
{
    return static_cast<double>(rand()) / RAND_MAX * (b - a) + a;
}

template <class T>
const T randomNormal(const T &sigma)
{
    T x1, x2, w;

    do {
        x1 = 2.0 * random(0.0, 1.0) - 1.0;
        x2 = 2.0 * random(0.0, 1.0) - 1.0;
        w = x1 * x1 + x2 * x2;
    } while (w >= 1.0 || w == 0.0);

    w = std::sqrt((-2.0 * log(w)) / w);

    return x1 * w * sigma;
}

void colorDepthImage(cv::Mat &imgDepth, cv::Mat &imgColoredDepth,
                     float minRange, float maxRange);

bool colormap(const std::string &name, unsigned char idx, float &r, float &g,
              float &b);

std::vector<cv::Point> bresLine(int x0, int y0, int x1, int y1);
std::vector<cv::Point> bresCircle(int x0, int y0, int r);

void fitCircle(const std::vector<cv::Point2d> &points, double &centerX,
               double &centerY, double &radius);

std::vector<cv::Point2d> intersectCircles(double x1, double y1, double r1,
                                          double x2, double y2, double r2);

} // namespace camodocal
