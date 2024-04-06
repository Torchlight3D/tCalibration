#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "display_profile.h"

enum class jpeg_app_t
{
    EXIF = 1,
    ICC = 2,
    NONE = 15
};

class Tiffsniff
{
public:
    explicit Tiffsniff(const std::string &fname, bool is_8bit = false);

    bool profile_found() const { return has_profile; }
    Display_profile profile();

private:
    enum class profile_t
    {
        sRGB,
        adobeRGB,
        UNKNOWN,
        CUSTOM
    };

    enum class ifd_t
    {
        ICC,
        TIFF,
        EXIF,
        EXIF_INTEROP
    };

    void parse_tiff(off_t offset);
    void read_ifd(off_t offset, off_t base_offset = 0,
                  ifd_t ifd_type = ifd_t::TIFF);
    void read_icc_profile(off_t offset);
    void read_trc_entry(off_t offset, uint32_t size);
    std::vector<double> read_xyztype_entry(off_t offset, uint32_t size);
    void read_curv_trc(off_t offset, uint32_t size);
    void read_para_trc(off_t offset);
    std::vector<std::pair<jpeg_app_t, off_t>> scan_jpeg_app_blocks();
    double read_exif_gamma(off_t offset);
    void parse_png(off_t offset);

    uint32_t read_uint32();
    uint16_t read_uint16();

    std::shared_ptr<std::iostream> fin;

    bool big_endian = false;
    bool has_profile = false;

    int exif_cs = -1;
    bool exif_gamma_found = false;
    bool exif_interop_r03 = false;

    profile_t inferred_profile = profile_t::UNKNOWN;
    off_t file_size;

    // linear gamma as default
    std::vector<double> gparm{1, 1, 0, 0, 0, 0, 0};
    std::vector<std::pair<uint16_t, uint16_t>> gtable;
    // sRGB RGB->Y adapted to D50 by default
    std::vector<double> luminance_weights{0.2225045, 0.7168786, 0.0606169};
};

struct tiff_field
{
    uint16_t tag_id;
    uint16_t data_type;
    uint32_t data_count;
    uint32_t data_offset;
};

struct icc_tag
{
    uint32_t tag_signature;
    uint32_t data_offset;
    uint32_t element_size;

    static double read_fixed8_8(std::shared_ptr<std::iostream> fin)
    {
        unsigned char b[2];
        fin->read((char *)b, 2);
        return double(b[0]) + double(b[1]) / 256.0;
    }

    static uint16_t read_uint16(std::shared_ptr<std::iostream> fin)
    {
        unsigned char b[2];
        fin->read((char *)b, 2);
        return (uint16_t(b[0]) << 8) | uint16_t(b[1]);
    }

    static double read_fixed15_16(std::shared_ptr<std::iostream> fin)
    {
        unsigned char b[4];
        fin->read((char *)b, 4);
        char sb0 = static_cast<char>(b[0]);
        return double((int16_t(sb0) << 8) | b[1]) +
               double(((uint16_t(b[2]) << 8) | b[3])) / 65536.0;
    }

    static uint32_t read_uint32(std::shared_ptr<std::iostream> fin)
    {
        unsigned char b[4];
        fin->read((char *)b, 4);
        return (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16) |
               (uint32_t(b[2]) << 8) | uint32_t(b[3]);
    }
};
