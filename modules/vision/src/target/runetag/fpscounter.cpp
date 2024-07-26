

#include "fpscounter.hpp"

FpsCounter::FpsCounter(float update_frequency)
    : update_frequency(update_frequency), num_frames(0), fps(1)
{
    timer.start();
}

float FpsCounter::getFps()
{
    if (timer.elapsed() > update_frequency) {
        fps = static_cast<float>(num_frames / timer.elapsed());
        timer.stop();
        timer.reset();
        timer.start();
        num_frames = 0;
    }
    return fps;
}

void FpsCounter::newframe() { num_frames++; }
