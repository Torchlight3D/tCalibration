#pragma once

#include <cstring>
#include <memory>

#include <opencv2/core/types.hpp>

#include "job_metadata.h"

class Edge_info
{
public:
    Edge_info() {}

    static bool serialize_header(FILE* fout, size_t valid_count,
                                 const Job_metadata& metadata);

    static bool deserialize_header(FILE* fin, size_t& valid_count,
                                   Job_metadata& metadata);

    // serialize multiple edges into a string
    static bool serialize(
        FILE* fout, const std::vector<cv::Point2d>& in_centroid,
        const std::vector<double>& in_angle,
        const std::vector<double>& in_mtf50,
        const std::vector<double>& /*in_quality*/,
        const std::vector<std::shared_ptr<std::vector<double>>>& in_sfr,
        const std::vector<std::shared_ptr<std::vector<double>>>& in_esf,
        const std::vector<cv::Point2d> in_snr,
        const std::vector<cv::Point2d> in_chromatic_aberration,
        const std::vector<bool> valid_edge,
        const std::vector<double>& in_edge_length,
        const std::vector<double>& in_geometric_length);

    // deserialize a single edge from string s
    static Edge_info deserialize(FILE* fin, bool& valid);

    void set_metadata(const Job_metadata& md);

public:
    cv::Point2d centroid;
    double angle;
    double mtf50;
    double quality;
    std::shared_ptr<std::vector<double>> sfr;
    std::shared_ptr<std::vector<double>> esf;
    cv::Point2d snr;
    cv::Point2d chromatic_aberration;
    Job_metadata metadata;
    double edge_length = 0;
    double geometric_edge_length = 0;

    static constexpr double nodata = -1073741824;

private:
    static uint32_t read_uint32(char** buffer);
    static void write_uint32(char** buffer, uint32_t val);

    static float read_float(char** buffer);
    static void write_float(char** buffer, float val);

    static double read_double(char** buffer);
    static void write_double(char** buffer, double val);
};
