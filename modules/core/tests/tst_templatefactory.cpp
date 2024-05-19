#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/TemplateFactory>

using namespace tl;

namespace tl {

class _Device : public Factory<_Device>
{
public:
    _Device(Key) {}

    virtual bool capture() = 0;
};

class _IMU : public _Device::Registry<_IMU>
{
public:
    _IMU() {}

    bool capture() override
    {
        LOG(INFO) << "tl::_IMU is capturing.";
        return true;
    }
};

} // namespace tl

namespace {

class _MonoCamera : public tl::_Device::Registry<_MonoCamera>
{
public:
    _MonoCamera() {}

    bool capture() override
    {
        LOG(INFO) << "(annoynomous namespace)::_MonoCamera is capturing.";
        return true;
    }
};

} // namespace

TEST(TemplateFactory, CreateFromName)
{
    auto imu = _Device::create("tl::_IMU");
    auto camera = _Device::create("_MonoCamera");
    EXPECT_TRUE(imu && imu->capture());
    // The correct class name should be (annoynomous namespace)::MonoCamera
    EXPECT_FALSE(camera && camera->capture());
}
