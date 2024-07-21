#include "visibilitypyramid.h"

#include <numeric>
#include <ranges>

#include <glog/logging.h>

namespace tl {

VisibilityPyramid::VisibilityPyramid(int width, int height, int pyramidLevel)
    : _width(width),
      _height(height),
      _pyramidLevel(pyramidLevel),
      _maxLayerDim(1 << pyramidLevel)
{
    CHECK_GT(_width, 0);
    CHECK_GT(_height, 0);
    CHECK_GT(_pyramidLevel, 0);

    // Create the occupancy pyramid such that the coarsest level is a 2x2 grid.
    _pyramid.resize(pyramidLevel);
    for (size_t i{0}; i < _pyramid.size(); ++i) {
        const auto layerDim = 1 << (1 + i);
        _pyramid[i].setZero(layerDim, layerDim);
    }
}

void VisibilityPyramid::addPoint(const Eigen::Vector2d& point)
{
    // Determine the grid cell of the point in the highest-resolution level of
    // the pyramid.
    int index_x =
        std::clamp(static_cast<int>(_maxLayerDim * point.x() / _width), 0,
                   _maxLayerDim - 1);
    int index_y =
        std::clamp(static_cast<int>(_maxLayerDim * point.y() / _height), 0,
                   _maxLayerDim - 1);

    // Go through the pyramid from fine to coarse and add the observation to the
    // occupancy grid.
    for (auto& level : std::ranges::reverse_view{_pyramid}) {
        level(index_x, index_y) += 1;

        // The next coarsest level of the pyramid will have half the number of
        // grid cells.
        index_x = index_x >> 1;
        index_y = index_y >> 1;
    }
}

int VisibilityPyramid::computeScore() const
{
    // The score is accumulated by counting the number of grid cells in each
    // level of the pyramid. The score for each level is weighted by the number
    // of grid cells in that level of the pyramid. This scheme favors good
    // spatial distribution at high resolutions.
    return std::accumulate(
        _pyramid.cbegin(), _pyramid.cend(), 0, [](int tmp, const auto& level) {
            return tmp + static_cast<int>(level.count() * level.size());
        });
}

} // namespace tl
