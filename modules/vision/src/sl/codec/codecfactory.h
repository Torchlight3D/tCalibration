#pragma once

#include "codec.h"

namespace tl {

enum class CodecType
{
    FastRatio,
    GrayCode,
    PhaseShift2p1,
    PhaseShift2x3,
    PhaseShift3,
    PhaseShift3Unwrap,
    PhaseShift3FastWrap,
    PhaseShift4,
    PhaseShiftDescatter,
    PhaseShiftMicro,
    PhaseShiftModulated,
    PhaseShiftNStep
};

class EncoderFactory
{
public:
    static std::unique_ptr<Encoder> NewEncoder(
        CodecType type, unsigned int screenResX, unsigned int screenResY,
        CodecDir dir = CodecDirHorizontal);
};

class DecoderFactory
{
public:
    static std::unique_ptr<Decoder> NewDecoder(
        CodecType type, unsigned int screenResX, unsigned int screenResY,
        CodecDir dir = CodecDirHorizontal);
};

} // namespace tl
