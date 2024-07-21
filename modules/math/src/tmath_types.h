#pragma once

namespace tl {

// For bundle adjustment we can use a robust cost function to maintain
// robustness to outliers. In particular, this function can help when feature
// tracks have outliers so that bundle adjustment may still optimize 3D points
// and camera poses properly without being catastrophically affected by
// outliers.
enum class LossFunctionType
{
    Trivial = 0,
    Huber,
    SoftLOne,
    Cauchy,
    Arctan,
    // Tolerant,
    Tukey,
    Truncated,
};

enum class RansacType
{
    RANSAC = 0,
    PROSAC = 1,
    LMED = 2,
    EXHAUSTIVE = 3
};

} // namespace tl
