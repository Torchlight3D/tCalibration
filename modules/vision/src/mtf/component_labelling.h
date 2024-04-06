#pragma once

#include "common_types.h"

// A class for extracting the boundaries of a binary image.
//
// It expects black objects on white backgrounds.
//
// Implements the method in (F. Chang, C.-J. Chen, C.-J. Jen, A linear-time
// component labeling algorithm using contour tracing technique, Computer
// Vision and Image Understanding, 93:206-220, 2004)

class Component_labeller
{
public:
    Component_labeller();
    Component_labeller(const cv::Mat &in_img, int min_boundary_length = 10,
                       bool snapshot = false, int max_boundary_length = 5000);

    ~Component_labeller();

    void release();

    void configure(const cv::Mat &in_img, int min_boundary_length = 10,
                   int max_boundary_length = 5000, bool snapshot = false);

    const Boundarylist &get_boundaries() const;

    int largest_hole(int label) const;

    int operator[](int index) const;
    int operator()(int x, int y) const;

    int get_width() const;
    int get_height() const;

    // set borders to the background color, since objects that
    // touch the image borders cause havoc with the main algorithm
    static void zap_borders(cv::Mat &masked_img, int fill = 255);

    void inflate_boundaries(double radius);

private:
    enum mode_type
    {
        INTERNAL = 0,
        EXTERNAL = 1,
        EXTERNAL_FIRST = 2,
        INTERNAL_FIRST = 3
    };

    void _find_components();

    void _contour_tracing(int x, int y, int label, mode_type mode);

    void _tracer(int x, int y, int &nx, int &ny, int &from, mode_type mode,
                 bool mark_white = true);

    void _draw_snapshot();

private:
    int _width;
    int _height;

    std::vector<uint8_t> _pix_data;
    uint8_t *_pix;
    std::vector<int32_t> _labels;
    Boundarylist _boundaries;
    std::map<int, int> _holes;

    int _min_boundary_length;
    int _max_boundary_length;

    int C;

    bool configured;
};
