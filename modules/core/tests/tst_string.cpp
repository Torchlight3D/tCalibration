#include <gtest/gtest.h>

#include <tCore/StringUtils>

using namespace tl;

TEST(String, ContainSubString)
{
    const std::string str{"HelloWorld"};
    const std::string sub1{"ello"};
    const std::string sub2{"low"};
    EXPECT_TRUE(str::Contains(str, sub1));
    EXPECT_FALSE(str::Contains(str, sub2));
}
