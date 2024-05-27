#pragma once

#include <map>
#include <memory>

#include "edge_model.h"
#include "rectangle.h"
#include "scanline.h"
#include "snr.h"

class Block
{
public:
    enum edge_position
    {
        TOP,
        LEFT,
        RIGHT,
        BOTTOM
    };

    Block();
    explicit Block(const Mrectangle &in_rect);

    void set_snr(size_t edge_number, const Snr &snr);
    const Snr &get_snr(size_t edge_number) const;

    void set_sfr(size_t edge_number, const std::vector<double> &sfr);
    const std::vector<double> &get_sfr(size_t edge_number) const;

    void set_esf(size_t edge_number, const std::vector<double> &esf);
    const std::vector<double> &get_esf(size_t edge_number) const;

    const std::vector<cv::Point2d> &get_ridge(size_t edge_number) const;

    void set_normal(size_t edge_number, const cv::Point2d &rgrad);
    cv::Point2d get_normal(size_t edge_number) const;

    cv::Point2d get_edge(size_t edge_number) const;

    cv::Point2d get_corner(size_t edge_number) const;

    double get_edge_angle(size_t edge_number) const;

    cv::Point2d get_edge_centroid(size_t edge_number) const;
    cv::Point2d get_edge_centroid(edge_position ep) const;

    // quality of 1.0 means good quality, 0.0 means unusably poor
    void set_mtf50_value(size_t edge_number, double mtf50_value,
                         double quality_value);
    double get_mtf50_value(size_t edge_number) const;
    double get_mtf50_value(edge_position ep) const;

    cv::Point2d get_centroid() const;

    double get_area() const;

    double get_quality(size_t edge_number) const;

    double get_quality(edge_position ep) const;

    int get_edge_index(edge_position ep) const;

    void set_line_deviation(size_t edge_number, const cv::Point3d &deviation);
    cv::Point3d get_line_deviation(size_t edge_number) const;

    void set_scanset(size_t edge_number,
                     const std::map<int, scanline> &scanset);
    const std::map<int, scanline> &get_scanset(size_t edge_number) const;

    void set_ca(size_t edge_number, cv::Point2d ca);
    const cv::Point2d &get_ca(size_t edge_number) const;

    void set_edge_model(size_t edge_number,
                        const std::shared_ptr<Edge_model> &em);
    Edge_model &get_edge_model(size_t edge_number);

    void set_edge_valid(size_t edge_number);
    bool get_edge_valid(size_t edge_number) const;

    void set_edge_length(size_t edge_number, double length);
    double get_edge_length(size_t edge_number) const;

    bool serialize(FILE *fout) const;

public:
    Mrectangle rect;
    std::vector<double> mtf50;
    std::vector<double> quality;
    std::vector<std::shared_ptr<std::vector<double>>> sfr;
    std::vector<std::shared_ptr<std::vector<double>>> esf;
    std::map<edge_position, size_t> edge_lut;
    cv::Point2d centroid;
    double area;
    bool valid;
    std::vector<cv::Point3d> line_deviation;
    std::vector<Snr> snr;
    std::vector<std::shared_ptr<std::map<int, scanline>>> scansets;
    std::vector<cv::Point2d> chromatic_aberration;
    std::vector<std::shared_ptr<Edge_model>> edge_model;
    std::vector<bool> valid_edge;
    std::vector<double> edge_length;
};
