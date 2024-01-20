#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "record_types.h"

namespace tl {

struct ImuData;

// TODO:
// 1. Keep consistent with StereoDataPlayer
// 2. Use already existed data structures
class StereoDataRecorder
{
public:
    using Ptr = std::unique_ptr<StereoDataRecorder>;
    using ConstPtr = std::unique_ptr<const StereoDataRecorder>;

    // Directory layout:
    // saveDir/left.avi
    // saveDir/left.csv
    // saveDir/right.avi
    // saveDir/right.csv
    // saveDir/motion.csv
    static Ptr create(const std::string &saveDir);

    ~StereoDataRecorder();

    /// Properties
    // Set reported fps for video recording. This does not affect what frame
    // data is actually recorded, only the FPS in the video file, which tells
    // how fast the video should be played.
    void setVideoRecordingFps(double fps);

    /// Motion
    void addMotionData(const ImuData &data);

    /// Visual
    bool addFrame(const FrameData &frame, bool deepCopy = true);
    bool addFrameGroup(double t, const std::vector<FrameData> &frames,
                       bool deepCopy = true);

    void closeOutputFile();

private:
    explicit StereoDataRecorder(const std::string &saveDir);

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

} // namespace tl
