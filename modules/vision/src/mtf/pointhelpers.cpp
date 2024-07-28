#include "pointhelpers.h"

#include "gradient.h"

cv::Point2d centroid(const Pointlist& points)
{
    cv::Point2d center;
    for (const auto& point : points) {
        center += point;
    }
    center /= static_cast<double>(points.size());

    return center;
}

cv::Point2d average_dir(const Gradient& g, int x, int y)
{
    double mx = 0;
    double my = 0;

    double wsum = 0;

    constexpr int offsets[][2]{{0, 0},  {-1, 0}, {1, 0},  {0, -1}, {0, 1},
                               {-1, 1}, {1, 1},  {1, -1}, {-1, -1}};

    for (int k = 0; k < 9; k++) {
        int lx = x + offsets[k][0];
        int ly = y + offsets[k][1];

        if (lx >= 0 && lx < g.width() && ly >= 0 && ly < g.height()) {
            mx += g.grad_x(lx, ly) * g.grad_magnitude(lx, ly);
            my += g.grad_y(lx, ly) * g.grad_magnitude(lx, ly);
            wsum += g.grad_magnitude(lx, ly);
        }
    }

    mx /= wsum;
    my /= wsum;
    return {-my, mx};
}

cv::Point2d normalize(const cv::Point2d& p)
{
    cv::Point2d q;
    double norm = sqrt(p.ddot(p));
    q.x = p.x / norm;
    q.y = p.y / norm;
    return q;
}
