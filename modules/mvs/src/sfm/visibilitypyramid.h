#pragma once

#include <Eigen/Core>

namespace tl {

// Brief:
// The main idea is that well-constrained views have a large number of
// observations and good spatial distribution of those observations. The view is
// divided into grid cells and a score is assigned to the grid based on the
// number of occupied cells. The score is accumulated over a pyrammid to capture
// coarse and fine-granded distribution statistics.
//
// Ref:
// [1] "Structure-from-Motion Revisited" by Johannes Schonberger and Jan-Michael
// Frahm (CVPR 2016)
// [2] colmap/colmap/blob/master/src/base/visibility_pyramid.h
class VisibilityPyramid
{
public:
    VisibilityPyramid(int width, int height, int pyramidLevel);

    void addPoint(const Eigen::Vector2d& point);

    // Higher scores indicate that the view is better constrained by the points.
    int computeScore() const;

private:
    const int _width, _height, _pyramidLevel, _maxLayerDim;
    // The pyramid represents all levels of image grids that keep track of the
    // features. The score of the pyramid may be efficiently computed with the
    // count() method that Eigen provides to determine occupancy.
    //
    // The pyramid is stored from coarse to fine and is indexed as (x, y).
    std::vector<Eigen::MatrixXi> _pyramid;
};

} // namespace tl
