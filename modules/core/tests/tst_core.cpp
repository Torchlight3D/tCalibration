#include <glog/logging.h>
#include <gtest/gtest.h>

#include <AxCore/ContainerUtils>
#include <AxCore/EnumUtils>
#include <AxCore/StringUtils>
#include <AxCore/TemplateFactory>

using namespace thoht;

TEST(StringTest, StringContain)
{
    std::string str{"HelloWorld"};
    std::string sub1{"ello"};
    std::string sub2{"low"};
    EXPECT_TRUE(str::Contains(str, sub1));
    EXPECT_FALSE(str::Contains(str, sub2));
}

namespace thoht {

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
        LOG(INFO) << "thoht::_IMU is capturing.";
        return true;
    }
};

} // namespace thoht

namespace {

class _MonoCamera : public thoht::_Device::Registry<_MonoCamera>
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

TEST(TemplateFactory, Creator)
{
    auto imu = _Device::create("thoht::_IMU");
    auto camera = _Device::create("_MonoCamera");
    EXPECT_TRUE(imu && imu->capture());
    // The correct class name should be (annoynomous namespace)::MonoCamera
    EXPECT_FALSE(camera && camera->capture());
}
