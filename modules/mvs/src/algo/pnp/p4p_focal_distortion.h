#pragma once

#include <tMath/EigenTypes>

namespace tl {

// Brief:
// Computes the absolute pose, focal length and one radial distortion parameter
// of a camera from 4 pairs of 2D-3D correspondences.
//
// Refernce paper:
// "Making Minimal Solvers for Absolute Pose Estimation Compact and Robust" by
// Larsson et al. (ICCV 2017)
// Matlab implementation:
// http://www.maths.lth.se/matematiklth/personal/viktorl/
//
// Input:
//     imagePoints: 4 vectors with image positions.
//     worldPoints: 4 3-vectors with corresponding 3D world points (each entry
//     is a point)
// Output:
//     rotations - world to camera rotations (R*X+t)
//     translations - world to camera translations (R*X+t)
//     radial_distortions - radial distortion corresponding to the "Division
//         Undistortion Camera Model"
//     focal_lengths - camera's focal length
// Return:
//     bool: returns if a solution was found

struct P4PFocalDistortionOptions
{
    double min_focal_length;
    double max_focal_length;

    // Smaller zero means we allow only barrel distortion for this model
    double min_distortion;
    double max_distortion;

    P4PFocalDistortionOptions(double minFocal, double maxFocal,
                              double minDistortion, double maxDistortion)
        : min_focal_length(minFocal),
          max_focal_length(maxFocal),
          min_distortion(minDistortion),
          max_distortion(maxDistortion)
    {
    }

    bool isValid() const;

    /// Helpers
    bool isValidFocalLength(double focalLength) const;
    bool isValidDistortion(double distortion) const;
};

bool FourPointsPoseFocalLengthRadialDistortion(
    const Vector2dList& imagePoints, const Vector3dList& worldPoints,
    const P4PFocalDistortionOptions& options, Matrix3dList* rotations,
    Vector3dList* translations, std::vector<double>* radialDistortions,
    std::vector<double>* focalLengths);

} // namespace tl
