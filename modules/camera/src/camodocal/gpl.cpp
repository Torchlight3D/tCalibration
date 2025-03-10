#include "gpl.h"

#ifdef _WIN32
#include <winsock.h>
#include <sys/types.h>
#include <winsock.h>
#else
#include <time.h>
#endif

namespace camodocal {

// TODO: Get rid of these old fashion interface
#ifdef _WIN32
LARGE_INTEGER getFILETIMEoffset()
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return (t);
}

int clock_gettime(int X, struct timespec *tp)
{
    LARGE_INTEGER t;
    FILETIME f;
    double microseconds;
    static LARGE_INTEGER offset;
    static double frequencyToMicroseconds;
    static int initialized = 0;
    static BOOL usePerformanceCounter = 0;

    if (!initialized) {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter =
            QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter) {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds =
                (double)performanceFrequency.QuadPart / 1000000.;
        }
        else {
            offset = getFILETIMEoffset();
            frequencyToMicroseconds = 10.;
        }
    }
    if (usePerformanceCounter) {
        QueryPerformanceCounter(&t);
    }
    else {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = (double)t.QuadPart / frequencyToMicroseconds;
    t.QuadPart = microseconds;
    tp->tv_sec = t.QuadPart / 1000000;
    tp->tv_nsec = (t.QuadPart % 1000000) * 1000;
    return (0);
}
#endif

double sinc(double theta) { return std::sin(theta) / theta; }

float colormapAutumn[128][3] = {
    {1.0f, 0.f, 0.f},       {1.0f, 0.007874f, 0.f}, {1.0f, 0.015748f, 0.f},
    {1.0f, 0.023622f, 0.f}, {1.0f, 0.031496f, 0.f}, {1.0f, 0.03937f, 0.f},
    {1.0f, 0.047244f, 0.f}, {1.0f, 0.055118f, 0.f}, {1.0f, 0.062992f, 0.f},
    {1.0f, 0.070866f, 0.f}, {1.0f, 0.07874f, 0.f},  {1.0f, 0.086614f, 0.f},
    {1.0f, 0.094488f, 0.f}, {1.0f, 0.10236f, 0.f},  {1.0f, 0.11024f, 0.f},
    {1.0f, 0.11811f, 0.f},  {1.0f, 0.12598f, 0.f},  {1.0f, 0.13386f, 0.f},
    {1.0f, 0.14173f, 0.f},  {1.0f, 0.14961f, 0.f},  {1.0f, 0.15748f, 0.f},
    {1.0f, 0.16535f, 0.f},  {1.0f, 0.17323f, 0.f},  {1.0f, 0.1811f, 0.f},
    {1.0f, 0.18898f, 0.f},  {1.0f, 0.19685f, 0.f},  {1.0f, 0.20472f, 0.f},
    {1.0f, 0.2126f, 0.f},   {1.0f, 0.22047f, 0.f},  {1.0f, 0.22835f, 0.f},
    {1.0f, 0.23622f, 0.f},  {1.0f, 0.24409f, 0.f},  {1.0f, 0.25197f, 0.f},
    {1.0f, 0.25984f, 0.f},  {1.0f, 0.26772f, 0.f},  {1.0f, 0.27559f, 0.f},
    {1.0f, 0.28346f, 0.f},  {1.0f, 0.29134f, 0.f},  {1.0f, 0.29921f, 0.f},
    {1.0f, 0.30709f, 0.f},  {1.0f, 0.31496f, 0.f},  {1.0f, 0.32283f, 0.f},
    {1.0f, 0.33071f, 0.f},  {1.0f, 0.33858f, 0.f},  {1.0f, 0.34646f, 0.f},
    {1.0f, 0.35433f, 0.f},  {1.0f, 0.3622f, 0.f},   {1.0f, 0.37008f, 0.f},
    {1.0f, 0.37795f, 0.f},  {1.0f, 0.38583f, 0.f},  {1.0f, 0.3937f, 0.f},
    {1.0f, 0.40157f, 0.f},  {1.0f, 0.40945f, 0.f},  {1.0f, 0.41732f, 0.f},
    {1.0f, 0.4252f, 0.f},   {1.0f, 0.43307f, 0.f},  {1.0f, 0.44094f, 0.f},
    {1.0f, 0.44882f, 0.f},  {1.0f, 0.45669f, 0.f},  {1.0f, 0.46457f, 0.f},
    {1.0f, 0.47244f, 0.f},  {1.0f, 0.48031f, 0.f},  {1.0f, 0.48819f, 0.f},
    {1.0f, 0.49606f, 0.f},  {1.0f, 0.50394f, 0.f},  {1.0f, 0.51181f, 0.f},
    {1.0f, 0.51969f, 0.f},  {1.0f, 0.52756f, 0.f},  {1.0f, 0.53543f, 0.f},
    {1.0f, 0.54331f, 0.f},  {1.0f, 0.55118f, 0.f},  {1.0f, 0.55906f, 0.f},
    {1.0f, 0.56693f, 0.f},  {1.0f, 0.5748f, 0.f},   {1.0f, 0.58268f, 0.f},
    {1.0f, 0.59055f, 0.f},  {1.0f, 0.59843f, 0.f},  {1.0f, 0.6063f, 0.f},
    {1.0f, 0.61417f, 0.f},  {1.0f, 0.62205f, 0.f},  {1.0f, 0.62992f, 0.f},
    {1.0f, 0.6378f, 0.f},   {1.0f, 0.64567f, 0.f},  {1.0f, 0.65354f, 0.f},
    {1.0f, 0.66142f, 0.f},  {1.0f, 0.66929f, 0.f},  {1.0f, 0.67717f, 0.f},
    {1.0f, 0.68504f, 0.f},  {1.0f, 0.69291f, 0.f},  {1.0f, 0.70079f, 0.f},
    {1.0f, 0.70866f, 0.f},  {1.0f, 0.71654f, 0.f},  {1.0f, 0.72441f, 0.f},
    {1.0f, 0.73228f, 0.f},  {1.0f, 0.74016f, 0.f},  {1.0f, 0.74803f, 0.f},
    {1.0f, 0.75591f, 0.f},  {1.0f, 0.76378f, 0.f},  {1.0f, 0.77165f, 0.f},
    {1.0f, 0.77953f, 0.f},  {1.0f, 0.7874f, 0.f},   {1.0f, 0.79528f, 0.f},
    {1.0f, 0.80315f, 0.f},  {1.0f, 0.81102f, 0.f},  {1.0f, 0.8189f, 0.f},
    {1.0f, 0.82677f, 0.f},  {1.0f, 0.83465f, 0.f},  {1.0f, 0.84252f, 0.f},
    {1.0f, 0.85039f, 0.f},  {1.0f, 0.85827f, 0.f},  {1.0f, 0.86614f, 0.f},
    {1.0f, 0.87402f, 0.f},  {1.0f, 0.88189f, 0.f},  {1.0f, 0.88976f, 0.f},
    {1.0f, 0.89764f, 0.f},  {1.0f, 0.90551f, 0.f},  {1.0f, 0.91339f, 0.f},
    {1.0f, 0.92126f, 0.f},  {1.0f, 0.92913f, 0.f},  {1.0f, 0.93701f, 0.f},
    {1.0f, 0.94488f, 0.f},  {1.0f, 0.95276f, 0.f},  {1.0f, 0.96063f, 0.f},
    {1.0f, 0.9685f, 0.f},   {1.0f, 0.97638f, 0.f},  {1.0f, 0.98425f, 0.f},
    {1.0f, 0.99213f, 0.f},  {1.0f, 1.0f, 0.0f}};

float colormapJet[128][3] = {
    {0.0f, 0.0f, 0.53125f},     {0.0f, 0.0f, 0.5625f},
    {0.0f, 0.0f, 0.59375f},     {0.0f, 0.0f, 0.625f},
    {0.0f, 0.0f, 0.65625f},     {0.0f, 0.0f, 0.6875f},
    {0.0f, 0.0f, 0.71875f},     {0.0f, 0.0f, 0.75f},
    {0.0f, 0.0f, 0.78125f},     {0.0f, 0.0f, 0.8125f},
    {0.0f, 0.0f, 0.84375f},     {0.0f, 0.0f, 0.875f},
    {0.0f, 0.0f, 0.90625f},     {0.0f, 0.0f, 0.9375f},
    {0.0f, 0.0f, 0.96875f},     {0.0f, 0.0f, 1.0f},
    {0.0f, 0.03125f, 1.0f},     {0.0f, 0.0625f, 1.0f},
    {0.0f, 0.09375f, 1.0f},     {0.0f, 0.125f, 1.0f},
    {0.0f, 0.15625f, 1.0f},     {0.0f, 0.1875f, 1.0f},
    {0.0f, 0.21875f, 1.0f},     {0.0f, 0.25f, 1.0f},
    {0.0f, 0.28125f, 1.0f},     {0.0f, 0.3125f, 1.0f},
    {0.0f, 0.34375f, 1.0f},     {0.0f, 0.375f, 1.0f},
    {0.0f, 0.40625f, 1.0f},     {0.0f, 0.4375f, 1.0f},
    {0.0f, 0.46875f, 1.0f},     {0.0f, 0.5f, 1.0f},
    {0.0f, 0.53125f, 1.0f},     {0.0f, 0.5625f, 1.0f},
    {0.0f, 0.59375f, 1.0f},     {0.0f, 0.625f, 1.0f},
    {0.0f, 0.65625f, 1.0f},     {0.0f, 0.6875f, 1.0f},
    {0.0f, 0.71875f, 1.0f},     {0.0f, 0.75f, 1.0f},
    {0.0f, 0.78125f, 1.0f},     {0.0f, 0.8125f, 1.0f},
    {0.0f, 0.84375f, 1.0f},     {0.0f, 0.875f, 1.0f},
    {0.0f, 0.90625f, 1.0f},     {0.0f, 0.9375f, 1.0f},
    {0.0f, 0.96875f, 1.0f},     {0.0f, 1.0f, 1.0f},
    {0.03125f, 1.0f, 0.96875f}, {0.0625f, 1.0f, 0.9375f},
    {0.09375f, 1.0f, 0.90625f}, {0.125f, 1.0f, 0.875f},
    {0.15625f, 1.0f, 0.84375f}, {0.1875f, 1.0f, 0.8125f},
    {0.21875f, 1.0f, 0.78125f}, {0.25f, 1.0f, 0.75f},
    {0.28125f, 1.0f, 0.71875f}, {0.3125f, 1.0f, 0.6875f},
    {0.34375f, 1.0f, 0.65625f}, {0.375f, 1.0f, 0.625f},
    {0.40625f, 1.0f, 0.59375f}, {0.4375f, 1.0f, 0.5625f},
    {0.46875f, 1.0f, 0.53125f}, {0.5f, 1.0f, 0.5f},
    {0.53125f, 1.0f, 0.46875f}, {0.5625f, 1.0f, 0.4375f},
    {0.59375f, 1.0f, 0.40625f}, {0.625f, 1.0f, 0.375f},
    {0.65625f, 1.0f, 0.34375f}, {0.6875f, 1.0f, 0.3125f},
    {0.71875f, 1.0f, 0.28125f}, {0.75f, 1.0f, 0.25f},
    {0.78125f, 1.0f, 0.21875f}, {0.8125f, 1.0f, 0.1875f},
    {0.84375f, 1.0f, 0.15625f}, {0.875f, 1.0f, 0.125f},
    {0.90625f, 1.0f, 0.09375f}, {0.9375f, 1.0f, 0.0625f},
    {0.96875f, 1.0f, 0.03125f}, {1.0f, 1.0f, 0.0f},
    {1.0f, 0.96875f, 0.0f},     {1.0f, 0.9375f, 0.0f},
    {1.0f, 0.90625f, 0.0f},     {1.0f, 0.875f, 0.0f},
    {1.0f, 0.84375f, 0.0f},     {1.0f, 0.8125f, 0.0f},
    {1.0f, 0.78125f, 0.0f},     {1.0f, 0.75f, 0.0f},
    {1.0f, 0.71875f, 0.0f},     {1.0f, 0.6875f, 0.0f},
    {1.0f, 0.65625f, 0.0f},     {1.0f, 0.625f, 0.0f},
    {1.0f, 0.59375f, 0.0f},     {1.0f, 0.5625f, 0.0f},
    {1.0f, 0.53125f, 0.0f},     {1.0f, 0.5f, 0.0f},
    {1.0f, 0.46875f, 0.0f},     {1.0f, 0.4375f, 0.0f},
    {1.0f, 0.40625f, 0.0f},     {1.0f, 0.375f, 0.0f},
    {1.0f, 0.34375f, 0.0f},     {1.0f, 0.3125f, 0.0f},
    {1.0f, 0.28125f, 0.0f},     {1.0f, 0.25f, 0.0f},
    {1.0f, 0.21875f, 0.0f},     {1.0f, 0.1875f, 0.0f},
    {1.0f, 0.15625f, 0.0f},     {1.0f, 0.125f, 0.0f},
    {1.0f, 0.09375f, 0.0f},     {1.0f, 0.0625f, 0.0f},
    {1.0f, 0.03125f, 0.0f},     {1.0f, 0.0f, 0.0f},
    {0.96875f, 0.0f, 0.0f},     {0.9375f, 0.0f, 0.0f},
    {0.90625f, 0.0f, 0.0f},     {0.875f, 0.0f, 0.0f},
    {0.84375f, 0.0f, 0.0f},     {0.8125f, 0.0f, 0.0f},
    {0.78125f, 0.0f, 0.0f},     {0.75f, 0.0f, 0.0f},
    {0.71875f, 0.0f, 0.0f},     {0.6875f, 0.0f, 0.0f},
    {0.65625f, 0.0f, 0.0f},     {0.625f, 0.0f, 0.0f},
    {0.59375f, 0.0f, 0.0f},     {0.5625f, 0.0f, 0.0f},
    {0.53125f, 0.0f, 0.0f},     {0.5f, 0.0f, 0.0f}};

void colorDepthImage(cv::Mat &imgDepth, cv::Mat &imgColoredDepth,
                     float minRange, float maxRange)
{
    imgColoredDepth = cv::Mat::zeros(imgDepth.size(), CV_8UC3);

    for (int i = 0; i < imgColoredDepth.rows; ++i) {
        const float *depth = imgDepth.ptr<float>(i);
        unsigned char *pixel = imgColoredDepth.ptr<unsigned char>(i);
        for (int j = 0; j < imgColoredDepth.cols; ++j) {
            if (depth[j] != 0) {
                int idx = fminf(depth[j] - minRange, maxRange - minRange) /
                          (maxRange - minRange) * 127.0f;
                idx = 127 - idx;

                pixel[0] = colormapJet[idx][2] * 255.0f;
                pixel[1] = colormapJet[idx][1] * 255.0f;
                pixel[2] = colormapJet[idx][0] * 255.0f;
            }

            pixel += 3;
        }
    }
}

bool colormap(const std::string &name, unsigned char idx, float &r, float &g,
              float &b)
{
    if (name.compare("jet") == 0) {
        float *color = colormapJet[idx];

        r = color[0];
        g = color[1];
        b = color[2];

        return true;
    }

    if (name.compare("autumn") == 0) {
        float *color = colormapAutumn[idx];

        r = color[0];
        g = color[1];
        b = color[2];

        return true;
    }

    return false;
}

// Bresenham's line algorithm
// Find cells intersected by line between (x0, y0) and (x1, y1)
std::vector<cv::Point> bresLine(int x0, int y0, int x1, int y1)
{
    std::vector<cv::Point> cells;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (1) {
        cells.emplace_back(x0, y0);

        if (x0 == x1 && y0 == y1) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return cells;
}

// Bresenham's circle algorithm
// Find cells intersected by circle with center (x0, y0) and radius r
std::vector<cv::Point> bresCircle(int x0, int y0, int r)
{
    // TODO: Use cv::Mat is much easier and faster
    const auto edge = 2 * r + 1;
    std::vector<std::vector<bool>> mask(edge);
    for (int i = 0; i < edge; ++i) {
        mask[i].resize(edge);
        for (int j = 0; j < edge; ++j) {
            mask[i][j] = false;
        }
    }

    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    std::vector<cv::Point> line;

    line = bresLine(x0, y0 - r, x0, y0 + r);
    for (const auto &point : line) {
        mask[point.x - x0 + r][point.y - y0 + r] = true;
    }

    line = bresLine(x0 - r, y0, x0 + r, y0);
    for (const auto &point : line) {
        mask[point.x - x0 + r][point.y - y0 + r] = true;
    }

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }

        x++;
        ddF_x += 2;
        f += ddF_x;

        line = bresLine(x0 - x, y0 + y, x0 + x, y0 + y);
        for (const auto &point : line) {
            mask[point.x - x0 + r][point.y - y0 + r] = true;
        }

        line = bresLine(x0 - x, y0 - y, x0 + x, y0 - y);
        for (const auto &point : line) {
            mask[point.x - x0 + r][point.y - y0 + r] = true;
        }

        line = bresLine(x0 - y, y0 + x, x0 + y, y0 + x);
        for (const auto &point : line) {
            mask[point.x - x0 + r][point.y - y0 + r] = true;
        }

        line = bresLine(x0 - y, y0 - x, x0 + y, y0 - x);
        for (const auto &point : line) {
            mask[point.x - x0 + r][point.y - y0 + r] = true;
        }
    }

    std::vector<cv::Point> cells;
    for (int i = 0; i < edge; ++i) {
        for (int j = 0; j < edge; ++j) {
            if (mask[i][j]) {
                cells.emplace_back(i - r + x0, j - r + y0);
            }
        }
    }

    return cells;
}

// D. Umbach, and K. Jones, A Few Methods for Fitting Circles to Data,
// IEEE Transactions on Instrumentation and Measurement, 2000
void fitCircle(const std::vector<cv::Point2d> &points, double &centerX,
               double &centerY, double &radius)
{
    // We use the modified least squares method.
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double sum_yy = 0.0;
    double sum_xxx = 0.0;
    double sum_xxy = 0.0;
    double sum_xyy = 0.0;
    double sum_yyy = 0.0;

    int n = points.size();
    for (int i = 0; i < n; ++i) {
        double x = points.at(i).x;
        double y = points.at(i).y;

        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_yy += y * y;
        sum_xxx += x * x * x;
        sum_xxy += x * x * y;
        sum_xyy += x * y * y;
        sum_yyy += y * y * y;
    }

    double A = n * sum_xx - square(sum_x);
    double B = n * sum_xy - sum_x * sum_y;
    double C = n * sum_yy - square(sum_y);
    double D =
        0.5 * (n * sum_xyy - sum_x * sum_yy + n * sum_xxx - sum_x * sum_xx);
    double E =
        0.5 * (n * sum_xxy - sum_y * sum_xx + n * sum_yyy - sum_y * sum_yy);

    centerX = (D * C - B * E) / (A * C - square(B));
    centerY = (A * E - B * D) / (A * C - square(B));

    double sum_r = 0.0;
    for (int i = 0; i < n; ++i) {
        double x = points.at(i).x;
        double y = points.at(i).y;

        sum_r += std::hypot(x - centerX, y - centerY);
    }

    radius = sum_r / n;
}

std::vector<cv::Point2d> intersectCircles(double x1, double y1, double r1,
                                          double x2, double y2, double r2)
{
    double d = std::hypot(x1 - x2, y1 - y2);
    if (d > r1 + r2) {
        // Circles are not overlap
        return {};
    }
    if (d < std::abs(r1 - r2)) {
        // One circle is inside the other
        return {};
    }

    double a = (square(r1) - square(r2) + square(d)) / (2.0 * d);
    double h = std::sqrt(square(r1) - square(a));

    double x3 = x1 + a * (x2 - x1) / d;
    double y3 = y1 + a * (y2 - y1) / d;

    std::vector<cv::Point2d> points;
    if (h < 1e-10) {
        // Two circles are in touch each other
        points.emplace_back(x3, y3);
        return points;
    }

    points.emplace_back(x3 + h * (y2 - y1) / d, y3 - h * (x2 - x1) / d);
    points.emplace_back(x3 - h * (y2 - y1) / d, y3 + h * (x2 - x1) / d);
    return points;
}

} // namespace camodocal
