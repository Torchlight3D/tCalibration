#pragma once

#include "hirestimer.hpp"

class FpsCounter
{
private:
    HiresTimer timer;
    float update_frequency;
    float last_update_time;
    float fps;
    unsigned int num_frames;

public:
    FpsCounter(float update_frequency = 3.0f);
    float getFps();
    void newframe();
};
