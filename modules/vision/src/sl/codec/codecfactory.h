#pragma once

#include <map>

#include "codec.h"

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

// TODO: Use magic_enum
static const std::map<const std::string, CodecType> Codecs{
    {"FastRatio", CodecType::FastRatio},
    {"GrayCode", CodecType::GrayCode},
    {"PhaseShift2p1", CodecType::PhaseShift2p1},
    {"PhaseShift2x3", CodecType::PhaseShift2x3},
    {"PhaseShift3", CodecType::PhaseShift3},
    {"PhaseShift3Unwrap", CodecType::PhaseShift3Unwrap},
    {"PhaseShift3FastWrap", CodecType::PhaseShift3FastWrap},
    {"PhaseShift4", CodecType::PhaseShift4},
    {"PhaseShiftDescatter", CodecType::PhaseShiftDescatter},
    {"PhaseShiftMicro", CodecType::PhaseShiftMicro},
    {"PhaseShiftModulated", CodecType::PhaseShiftModulated},
    {"PhaseShiftNStep", CodecType::PhaseShiftNStep}};

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
