#pragma once

class Mtf_core;
class Stride_range;

class Mtf_core_tbb_adaptor
{
public:
    Mtf_core_tbb_adaptor(Mtf_core *core);

    void operator()(const Stride_range &r) const;

public:
    Mtf_core *mtf_core;
};
