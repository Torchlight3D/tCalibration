#include "display_profile.h"
#include "logger.h"

#include <ctime>

Display_profile::Display_profile() : luminance_weights{0.2126, 0.7152, 0.0722}
{
    is_linear = true;
}

Display_profile::Display_profile(const std::vector<double>& gparm,
                                 const std::vector<double>& luminance_weights)
    : luminance_weights(luminance_weights)
{
    render_parametric(gparm);
}

Display_profile::Display_profile(
    const std::vector<std::pair<uint16_t, uint16_t>>& gtable,
    const std::vector<double>& luminance_weights)
    : luminance_weights(luminance_weights)
{
    for (size_t i = 0; i < gtable.size() - 1; i++) {
        size_t start_x = gtable[i].first;
        size_t end_x = gtable[i + 1].first;
        double slope =
            (gtable[i + 1].second - gtable[i].second) / double(end_x - start_x);
        double offset = gtable[i].second - slope * gtable[i].first;
        for (size_t j = start_x; j <= end_x; j++) {
            lut[j] = int(j * slope + offset);
        }
    }
}

void Display_profile::force_linear() { is_linear = true; }

void Display_profile::force_sRGB()
{
    std::vector<double> gparm{
        2.4, 1.0 / 1.055, 0.055 / 1.055, 1.0 / 12.92, 0.04045, 0, 0};

    luminance_weights[0] = 0.2126;
    luminance_weights[1] = 0.7152;
    luminance_weights[2] = 0.0722;

    render_parametric(gparm);
}

// this ungainly function is about 4x faster than using cv::split(),
// and it does not waste any memory, so we have to live with the
// repetitive mess
cv::Mat Display_profile::to_luminance(const cv::Mat& img)
{
    if (!is_linear) {
#ifdef MDEBUG
        FILE* fout = fopen("final_trc.txt", "wt");
        for (size_t i = 0; i < lut.size(); i++) {
            fprintf(fout, "%lf %lf\n", i / 65535.0, lut[i] / 65535.0);
        }
        fclose(fout);
#endif

        if (img.elemSize1() == 1) {
            cv::Mat newmat = cv::Mat(img.rows, img.cols, CV_16UC1);
            uint16_t* dptr = (uint16_t*)newmat.data;
            uint8_t* sptr = (uint8_t*)img.data;
            if (img.channels() == 3) {
                logger.info(
                    "8-bit nonlinear RGB, luma=%.3lfR + %.3lfG + %.3lfB\n",
                    luminance_weights[0], luminance_weights[1],
                    luminance_weights[2]);
                uint8_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    double gray = 0.5;
                    gray += luminance_weights[2] * lut[(*sptr++ << 8)];
                    gray += luminance_weights[1] * lut[(*sptr++ << 8)];
                    gray += luminance_weights[0] * lut[(*sptr++ << 8)];

                    *dptr++ = uint16_t(gray);
                }
                return newmat;
            }
            else {
                logger.info("%s\n", "8-bit nonlinear grayscale");
                uint8_t* sentinel = sptr + img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr++ = uint16_t(lut[(*sptr++ << 8)]);
                }
                return newmat;
            }
        }
        else {
            if (img.channels() == 3) {
                logger.info(
                    "16-bit nonlinear RGB, luma=%.3lfR + %.3lfG + %.3lfB\n",
                    luminance_weights[0], luminance_weights[1],
                    luminance_weights[2]);
                cv::Mat newmat = cv::Mat(img.rows, img.cols, CV_16UC1);
                uint16_t* dptr = (uint16_t*)newmat.data;
                uint16_t* sptr = (uint16_t*)img.data;
                uint16_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    double gray = 0.5;
                    gray += luminance_weights[2] * lut[*sptr++];
                    gray += luminance_weights[1] * lut[*sptr++];
                    gray += luminance_weights[0] * lut[*sptr++];

                    *dptr++ = uint16_t(gray);
                }
                return newmat;
            }
            else {
                logger.info("%s\n", "16-bit nonlinear grayscale");
                uint16_t* dptr = (uint16_t*)img.data;
                uint16_t* sptr = (uint16_t*)img.data;

                uint16_t* sentinel = sptr + img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr++ = uint16_t(lut[*sptr++]);
                }
                return img;
            }
        }
    }
    else {
        if (img.elemSize1() == 1) {
            cv::Mat newmat = cv::Mat(img.rows, img.cols, CV_16UC1);
            uint16_t* dptr = (uint16_t*)newmat.data;
            uint8_t* sptr = (uint8_t*)img.data;
            if (img.channels() == 3) {
                logger.info("8-bit linear RGB, luma=%.3lfR + %.3lfG + %.3lfB\n",
                            luminance_weights[0], luminance_weights[1],
                            luminance_weights[2]);
                uint8_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    double gray = 0.5;
                    gray += luminance_weights[2] * (*sptr++ << 8);
                    gray += luminance_weights[1] * (*sptr++ << 8);
                    gray += luminance_weights[0] * (*sptr++ << 8);

                    *dptr++ = uint16_t(gray);
                }
                return newmat;
            }
            else {
                logger.info("%s\n", "8-bit linear grayscale");
                uint8_t* sentinel = sptr + img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr++ = uint16_t(*sptr++ << 8);
                }
                return newmat;
            }
        }
        else {
            if (img.channels() == 3) {
                logger.info(
                    "16-bit linear RGB, luma=%.3lfR + %.3lfG + %.3lfB\n",
                    luminance_weights[0], luminance_weights[1],
                    luminance_weights[2]);
                cv::Mat newmat = cv::Mat(img.rows, img.cols, CV_16UC1);
                uint16_t* dptr = (uint16_t*)newmat.data;
                uint16_t* sptr = (uint16_t*)img.data;
                uint16_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    double gray = 0.5;
                    gray += luminance_weights[2] * (*sptr++);
                    gray += luminance_weights[1] * (*sptr++);
                    gray += luminance_weights[0] * (*sptr++);

                    *dptr++ = uint16_t(gray);
                }
                return newmat;
            }
            else {
                logger.info("%s\n", "16-bit linear grayscale");
                return img;
            }
        }
    }
}

std::vector<cv::Mat> Display_profile::to_linear_rgb(const cv::Mat& img)
{
    // regardless of input type, output is always 16-bit linear
    std::vector<cv::Mat> newmat;
    for (int i = 0; i < 3; i++) {
        newmat.push_back(cv::Mat(img.rows, img.cols, CV_16UC1));
    }
    uint16_t* dptr_r = (uint16_t*)newmat[2].data;
    uint16_t* dptr_g = (uint16_t*)newmat[1].data;
    uint16_t* dptr_b = (uint16_t*)newmat[0].data;

    if (!is_linear) {
#ifdef MDEBUG
        FILE* fout = fopen("final_trc.txt", "wt");
        for (size_t i = 0; i < lut.size(); i++) {
            fprintf(fout, "%lf %lf\n", i / 65535.0, lut[i] / 65535.0);
        }
        fclose(fout);
#endif

        if (img.elemSize1() == 1) {
            uint8_t* sptr = (uint8_t*)img.data;
            if (img.channels() == 3) {
                logger.info("%s\n", "8-bit nonlinear RGB -> 16-bit linear RGB");
                uint8_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(lut[(*sptr++ << 8)]);
                    *dptr_g++ = uint16_t(lut[(*sptr++ << 8)]);
                    *dptr_r++ = uint16_t(lut[(*sptr++ << 8)]);
                }
            }
            else {
                logger.info(
                    "%s\n",
                    "8-bit nonlinear grayscale -> 16-bit linear gray RGB");
                uint8_t* sentinel = sptr + img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(lut[(*sptr << 8)]);
                    *dptr_g++ = uint16_t(lut[(*sptr << 8)]);
                    *dptr_r++ = uint16_t(lut[(*sptr++ << 8)]);
                }
            }
        }
        else {
            if (img.channels() == 3) {
                logger.info("%s\n",
                            "16-bit nonlinear RGB -> 16-bit linear RGB");
                uint16_t* sptr = (uint16_t*)img.data;
                uint16_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(lut[*sptr++]);
                    *dptr_g++ = uint16_t(lut[*sptr++]);
                    *dptr_r++ = uint16_t(lut[*sptr++]);
                }
            }
            else {
                logger.info(
                    "%s\n",
                    "16-bit nonlinear grayscale -> 16-bit linear gray RGB");
                uint16_t* sptr = (uint16_t*)img.data;

                uint16_t* sentinel = sptr + img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(lut[*sptr]);
                    *dptr_g++ = uint16_t(lut[*sptr]);
                    *dptr_r++ = uint16_t(lut[*sptr++]);
                }
            }
        }
    }
    else {
        if (img.elemSize1() == 1) {
            uint8_t* sptr = (uint8_t*)img.data;
            if (img.channels() == 3) {
                logger.info("%s\n", "8-bit linear RGB -> 16-bit linear RGB");
                uint8_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(*sptr++ << 8);
                    *dptr_g++ = uint16_t(*sptr++ << 8);
                    *dptr_r++ = uint16_t(*sptr++ << 8);
                }
            }
            else {
                logger.info("%s\n",
                            "8-bit linear grayscale -> 16-bit linear gray RGB");
                uint8_t* sentinel = sptr + img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(*sptr << 8);
                    *dptr_g++ = uint16_t(*sptr << 8);
                    *dptr_r++ = uint16_t(*sptr++ << 8);
                }
            }
        }
        else {
            if (img.channels() == 3) {
                logger.info("%s\n", "16-bit linear RGB -> 16-bit linear RGB "
                                    "(separate channels)");
                uint16_t* sptr = (uint16_t*)img.data;
                uint16_t* sentinel = sptr + 3 * img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(*sptr++);
                    *dptr_g++ = uint16_t(*sptr++);
                    *dptr_r++ = uint16_t(*sptr++);
                }
            }
            else {
                logger.info(
                    "%s\n",
                    "16-bit linear grayscale -> 16-bit linear gray RGB");
                uint16_t* sptr = (uint16_t*)img.data;
                uint16_t* sentinel = sptr + img.rows * img.cols;
                while (sptr < sentinel) {
                    *dptr_b++ = uint16_t(*sptr);
                    *dptr_g++ = uint16_t(*sptr);
                    *dptr_r++ = uint16_t(*sptr++);
                }
            }
        }
    }

    return newmat;
}

void Display_profile::render_parametric(const std::vector<double>& gparm)
{
    is_linear = (gparm[0] == 1.0) && (gparm[1] == 1.0);
    for (size_t i = 2; i < gparm.size(); i++) {
        is_linear &= (gparm[i] == 0.0);
    }

    if (!is_linear) {
        for (size_t i = 0; i < lut.size(); i++) {
            double y;
            double x = i / 65535.0;
            if (x < gparm[4]) {
                y = gparm[3] * x + gparm[6];
            }
            else {
                y = pow(x * gparm[1] + gparm[2], gparm[0]) + gparm[5];
            }
            lut[i] = int(rint(y * 65535)) & 0xffff;
        }
    }
}
