#pragma once

#include <tCore/Math>

namespace orp {
namespace calibration {

struct DetectorParams
{
    /************************************/
    /* saddle point localization params */
    /************************************/

    // maximum dimension of the image for the initial localization of the
    // points, bigger images are downscaled to this resolution before running
    // the saddle point localization points found are then re
    const double working_resolution = 3000.0;

    // size of the polynomial fitting window (2 x half_kernel_size + 1)
    const int half_kernel_size = 3;

    // fraction of the maximum saddle determinant used to prefilter regular
    // saddles
    const double rectangular_saddle_threshold = 1.0 / 20;

    // size increase of the polynomial fitting window for the second pass
    // verification of deltille points
    const int deltille_stability_kernel_size_increase = 1;

    // maximum movement of the point allowed in the multi-scale verification
    const double deltille_stability_threshold = 0.1;

    // spatial convergence threshold, minimum change in x or y coordinate
    // required to continue in fitting
    const double spatial_convergence_threshold = 0.001;

    /************************************/
    /* Grid indexing parameters         */
    /************************************/

    // maximum number of nearest neighbours considered in the grid growing
    const int max_nearest_neighbours = 50;

    // half of the angular span (-shadow_angle, shadow_angle) "shadow" that will
    // be cast from each point in grid growing
    const double shadow_angle = tl::math::degToRad(15.0);

    // maximum angular difference accepted to consider polarities "same" or
    // "different"
    const double rectangle_polarity_angle_threshold = tl::math::degToRad(30.);

    // maximum angular difference accepted to consider polarities "same" for
    // triangular grid
    const double triangle_polarity_angle_threshold = tl::math::degToRad(20.);

    // maximum intensity difference accepted for an initial quad selection
    const double rectangle_consistency_threshold = 0.1 * 255.0;

    // maximum intensity difference accepted for an initial deltille quad
    // sellection
    const double triangle_consistency_threshold = 0.1 * 255.0;

    // maximum number of initial quad selection attempts per grid growing
    const int grid_search_max_iterations = 200;

    // number of initial quads searched in grid growing
    const int grid_search_quad_growing_trials = 10;

    /************************************/
    /* Grid growing parameters          */
    /************************************/

    // minimum and maximum bounds for the circle fit edge checking (in px)
    double delta_r_min = 2.0;
    double delta_r_max = 5.0;

    // relative distance of the circular edge checking
    double delta_r_rel = 0.05;

    // begining of the edge stability search interval
    const double edge_stability_interval_min = 0.2;
    const double edge_stability_interval_max = 0.8;
    const double edge_stability_interval_step = 0.01;

    // minimum required intensity difference in edge checking/growing (against
    // the sign flips along the edge)
    const double edge_stability_threshold = 0.05 * 255.0;

    // maximum intensity difference between the maximum and minimum observed
    // along the edge growing
    const double edge_consistency_threshold = 0.2 * 255.0;
};

extern DetectorParams detector_params;
} // namespace calibration
} // namespace orp
