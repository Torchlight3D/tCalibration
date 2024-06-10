#include <gtest/gtest.h>

#include <tCamera/LinearCameraResponse>
#include <tCamera/PolyCameraResponse>
#include <tCamera/DenseVignetting>
#include <tCamera/PolyVignetting>
#include <tCamera/UniformVignetting>

using namespace tl;

TEST(CameraResponse, Specialization)
{
    auto specialize = []() {
        LinearResponse res1;
        Poly3Response res2;
        Poly4Response res3;
    };

    EXPECT_NO_FATAL_FAILURE(specialize());
}

TEST(Vignetting, Specialization)
{
    auto specialize = []() {
        UniformVignetting res1{600, 400};
        EvenPoly6Vignetting res2{600, 400};
        DenseVignetting res3{600, 400};
    };

    EXPECT_NO_FATAL_FAILURE(specialize());
}
