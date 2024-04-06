#pragma once

#include <map>

#include "afft.h"
#include "bayer.h"
#include "block.h"
#include "component_labelling.h"
#include "ellipse.h"
#include "esf_model.h"
#include "esf_sampler.h"
#include "gradient.h"
#include "mtf_profile_sample.h"
#include "rectangle.h"
#include "sampling_rate.h"
#include "snr.h"
#include "undistort.h"

using block_vector = std::vector<Block>;

// global constants for ESF-fourier MTF method
// TODO: these can be dynamic parameters, with some effort
constexpr double max_dot = 28;

class Mtf_core
{
public:
    Mtf_core(const Component_labeller &in_cl, const Gradient &in_g,
             const cv::Mat &in_img, const cv::Mat &in_bayer_img,
             std::string bayer_subset, std::string cfa_pattern_name,
             std::string esf_sampler_name, Undistort *undistort = nullptr,
             int border_width = 0);

    ~Mtf_core();

    size_t num_objects();

    void search_borders(const cv::Point2d &cent, int label);
    bool extract_rectangle(const cv::Point2d &cent, int label,
                           Mrectangle &rect);
    double compute_mtf(Edge_model &edge_model,
                       const std::map<int, scanline> &scanset, double &poor,
                       double &edge_length, std::vector<double> &sfr,
                       std::vector<double> &esf, Snr &snr,
                       bool allow_peak_shift = false);

    std::vector<Block> &get_blocks();
    std::vector<Mtf_profile_sample> &get_samples();

    void set_absolute_sfr(bool val);

    void set_sfr_smoothing(bool val);

    void set_sliding(bool val);

    void set_samples_per_edge(int s);

    void set_snap_angle(double angle);

    void set_find_fiducials(bool val);

    void set_ridges_only(bool b);

    void use_full_sfr();

    void set_mtf_contrast(double contrast);

    double get_mtf_contrast() const;

    void process_image_as_roi(const cv::Rect &bounds,
                              cv::Point2d handle_a = {-1e11, -1e11},
                              cv::Point2d handle_b = {-1e11, -1e11});

    void process_manual_rois(const std::string &roi_fname);

    void set_esf_model(std::unique_ptr<EsfModel> &&model);

    std::unique_ptr<EsfModel> &get_esf_model();

    void set_esf_model_alpha_parm(double alpha);

    const std::vector<std::pair<cv::Point2d, cv::Point2d>> &get_sliding_edges()
        const;

    EsfSampler *get_esf_sampler() const;

    Bayer::cfa_pattern_t get_cfa_pattern() const;

    bool is_single_roi() const;

    void set_allow_partial(bool value);

    const Component_labeller &cl;
    const Gradient &g;
    const cv::Mat &img;
    const cv::Mat &bayer_img;
    Bayer::bayer_t bayer;
    Bayer::cfa_pattern_t cfa_pattern;

    AFFT<512> afft; // FFT_SIZE = 512 ??
    std::vector<int> valid_obj;

    std::vector<Block> detected_blocks;
    std::map<int, Block> shared_blocks_map;
    std::vector<cv::Point2d> solid_ellipses;
    std::vector<Ellipse_detector> ellipses;

    std::vector<Mtf_profile_sample> samples;

    cv::Mat od_img;

#ifdef MDEBUG
    double noise_seed = 10;
    double noise_sd = 0;
#endif

private:
    bool absolute_sfr;
    bool snap_to;
    double snap_to_angle;
    bool sfr_smoothing;
    bool sliding;
    int samples_per_edge;
    bool find_fiducials;
    Undistort *undistort = nullptr;
    bool ridges_only;
    size_t mtf_width = 2 * NYQUIST_FREQ;
    EsfSampler *esf_sampler = nullptr;
    double mtf_contrast = 0.5; // target MTF contrast, e.g., 0.5 -> MTF50
    std::unique_ptr<EsfModel> esf_model;
    std::vector<std::pair<cv::Point2d, cv::Point2d>> sliding_edges;

    void process_with_sliding_window(Mrectangle &rrect);
    bool homogenous(const cv::Point2d &cent, int label,
                    const Mrectangle &rrect) const;
    bool single_roi_mode = false;
    bool allow_partial = false;
};
