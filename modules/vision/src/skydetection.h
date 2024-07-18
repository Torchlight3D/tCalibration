#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {

class SkyAreaDetector
{
public:
    SkyAreaDetector() = default;
    ~SkyAreaDetector() = default;

    SkyAreaDetector(const SkyAreaDetector &);
    SkyAreaDetector &operator=(const SkyAreaDetector &);

    void detect(const std::string &image_file_path,
                const std::string &output_path);

    void batch_detect(const std::string &image_dir,
                      const std::string &output_dir);

private:
    cv::Mat _src_img;

    double f_thres_sky_max = 600;
    double f_thres_sky_min = 5;
    double f_thres_sky_search_step = 5;

    bool load_image(const std::string &image_file_path);

    bool extract_sky(const cv::Mat &src_image, cv::Mat &sky_mask);

    void extract_image_gradient(const cv::Mat &src_image,
                                cv::Mat &gradient_image);

    std::vector<int> extract_border_optimal(const cv::Mat &src_image);

    std::vector<int> extract_border(const cv::Mat &gradient_info_map,
                                    double thresh);

    std::vector<int> refine_border(const std::vector<int> &border,
                                   const cv::Mat &src_image);

    double calculate_sky_energy(const std::vector<int> &border,
                                const cv::Mat &src_image);

    bool has_sky_region(const std::vector<int> &border, double thresh_1,
                        double thresh_2, double thresh_3);

    bool has_partial_sky_region(const std::vector<int> &border,
                                double thresh_1);

    void display_sky_region(const cv::Mat &src_image,
                            const std::vector<int> &border, cv::Mat &sky_image);

    cv::Mat make_sky_mask(const cv::Mat &src_image,
                          const std::vector<int> &border, int type = 1);
};

} // namespace tl
