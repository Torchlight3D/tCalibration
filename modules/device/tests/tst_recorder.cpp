#include <gtest/gtest.h>

#include <ostream>

#include <AxDevice/StereoDataRecorder>

using namespace thoht;

TEST(Recorder, RecordData)
{
    // Write to file:
    // auto r = Recorder::build("test.json");
    // Write to stdout:
    // auto r = Recorder::build(std::cout);

    std::ostringstream output;
    // FIXME: No hardcode here
    auto record = StereoDataRecorder::create(std::string{"SavedData"});

    FrameData f0{.t = 0.,
                 .focalLengthX = 1000.0,
                 .focalLengthY = 1000.0,
                 .px = 640.0,
                 .py = 360.0,
                 .cameraId = 0};
    FrameData f1{.t = 0.1,
                 .focalLengthX = 1001.0,
                 .focalLengthY = 1001.0,
                 .px = 640.0,
                 .py = 360.0,
                 .cameraId = 1};

    record->addFrame(f0);
    record->addFrame(f1);

    record->addFrameGroup(0.15, {f0, f1});
    record->addFrameGroup(0.20, {f1});
    record->addFrameGroup(0.25, {f1, f0});
}
