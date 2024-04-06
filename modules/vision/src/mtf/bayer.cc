#include "bayer.h"

std::string Bayer::to_string(bayer_t bayer)
{
    std::string bname = "";
    switch (bayer) {
        case RED:
            bname = "red";
            break;
        case BLUE:
            bname = "blue";
            break;
        case GREEN:
            bname = "green";
            break;
        default:
            bname = "";
            break;
    }
    return bname;
}

Bayer::bayer_t Bayer::from_string(const std::string &bayer_subset)
{
    if (bayer_subset.compare("none") == 0) {
        return NONE;
    }
    if (bayer_subset.compare("red") == 0) {
        return RED;
    }
    if (bayer_subset.compare("blue") == 0) {
        return BLUE;
    }
    if (bayer_subset.compare("green") == 0) {
        return GREEN;
    }
    return NONE;
}

Bayer::cfa_pattern_t Bayer::from_cfa_string(const std::string &cfa_pattern)
{
    if (cfa_pattern.compare("rggb") == 0) {
        return cfa_pattern_t::RGGB;
    }
    if (cfa_pattern.compare("bggr") == 0) {
        return cfa_pattern_t::BGGR;
    }
    if (cfa_pattern.compare("grbg") == 0) {
        return cfa_pattern_t::GRBG;
    }
    if (cfa_pattern.compare("gbrg") == 0) {
        return cfa_pattern_t::GBRG;
    }
    return cfa_pattern_t::RGGB;
}

Bayer::cfa_mask_t Bayer::to_cfa_mask(bayer_t subset, cfa_pattern_t cfa_pattern)
{
    switch (cfa_pattern) {
        case RGGB:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                    break;
                case RED:
                    return cfa_mask_t::RGGB_RED;
                    break;
                case GREEN:
                    return cfa_mask_t::RGGB_GREEN;
                    break;
                case BLUE:
                    return cfa_mask_t::RGGB_BLUE;
                    break;
            }
            break;
        case BGGR:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                    break;
                case RED:
                    return cfa_mask_t::BGGR_RED;
                    break;
                case GREEN:
                    return cfa_mask_t::BGGR_GREEN;
                    break;
                case BLUE:
                    return cfa_mask_t::BGGR_BLUE;
                    break;
            }
            break;
        case GRBG:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                    break;
                case RED:
                    return cfa_mask_t::GRBG_RED;
                    break;
                case GREEN:
                    return cfa_mask_t::GRBG_GREEN;
                    break;
                case BLUE:
                    return cfa_mask_t::GRBG_BLUE;
                    break;
            }
            break;
        case GBRG:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                    break;
                case RED:
                    return cfa_mask_t::GBRG_RED;
                    break;
                case GREEN:
                    return cfa_mask_t::GBRG_GREEN;
                    break;
                case BLUE:
                    return cfa_mask_t::GBRG_BLUE;
                    break;
            }
            break;
    }
    return cfa_mask_t::ALL;
}
