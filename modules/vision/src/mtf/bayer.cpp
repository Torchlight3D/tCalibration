#include "bayer.h"

namespace Bayer {

// TODO: This is basically making combination of two enums, try better
cfa_mask_t to_cfa_mask(bayer_t subset, cfa_pattern_t cfa_pattern)
{
    switch (cfa_pattern) {
        case RGGB:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                case RED:
                    return cfa_mask_t::RGGB_RED;
                case GREEN:
                    return cfa_mask_t::RGGB_GREEN;
                case BLUE:
                    return cfa_mask_t::RGGB_BLUE;
                default:
                    break;
            }
            break;
        case BGGR:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                case RED:
                    return cfa_mask_t::BGGR_RED;
                case GREEN:
                    return cfa_mask_t::BGGR_GREEN;
                case BLUE:
                    return cfa_mask_t::BGGR_BLUE;
                default:
                    break;
            }
            break;
        case GRBG:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                case RED:
                    return cfa_mask_t::GRBG_RED;
                case GREEN:
                    return cfa_mask_t::GRBG_GREEN;
                case BLUE:
                    return cfa_mask_t::GRBG_BLUE;
                default:
                    break;
            }
            break;
        case GBRG:
            switch (subset) {
                case NONE:
                    return cfa_mask_t::ALL;
                case RED:
                    return cfa_mask_t::GBRG_RED;
                case GREEN:
                    return cfa_mask_t::GBRG_GREEN;
                case BLUE:
                    return cfa_mask_t::GBRG_BLUE;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    return cfa_mask_t::ALL;
}

} // namespace Bayer
