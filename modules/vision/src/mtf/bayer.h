#pragma once

namespace Bayer {

// WARNING: The default stringify style is all lower case letters

enum bayer_t
{
    NONE,
    RED,
    GREEN,
    BLUE
};

// mask layout, in pixels:
// A | B
// C | D
// => A<<3 | B << 2 | C << 1 | D
enum cfa_mask_t
{
    DEFAULT = 0x1f, // placeholder used as default argument to samplers
    ALL = 0xf,

    RGGB_RED = 1 << 3,
    RGGB_GREEN = (1 << 2) | (1 << 1),
    RGGB_BLUE = 1,

    BGGR_RED = 1,
    BGGR_GREEN = (1 << 2) | (1 << 1),
    BGGR_BLUE = 1 << 3,

    GRBG_RED = 1 << 2,
    GRBG_GREEN = (1 << 3) | 1,
    GRBG_BLUE = 1 << 1,

    GBRG_RED = 1 << 1,
    GBRG_GREEN = (1 << 3) | 1,
    GBRG_BLUE = 1 << 2
};

// TODO: add some more cfa pattern types
enum cfa_pattern_t
{
    RGGB,
    BGGR,
    GRBG,
    GBRG
};

cfa_mask_t to_cfa_mask(bayer_t subset, cfa_pattern_t cfa_pattern = RGGB);

}; // namespace Bayer
