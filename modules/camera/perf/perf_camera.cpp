#include <benchmark/benchmark.h>

#include <tCamera/coc/CataCamera>
#include <tCamera/coc/EquidistantCamera>
#include <tCamera/FisheyeCameraModel>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCore/Global>
#include <tCore/RandomGenerator>

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace bm = ::benchmark;
using namespace tl;

class NearViewFixture : public bm::Fixture
{
public:
    NearViewFixture()
        : m_camera{"camera",  1280,      800,         0.894975,
                   -0.344504, 0.0984552, -0.00403995, 0.00610364,
                   758.355,   757.615,   646.72,      395.001},
          w1{0.3},
          l1{0.2},
          w2{0.9},
          l2{0.6},
          h{0.6}
    {
    }

    void SetUp(bm::State &state) override
    {
        //
    }

    void TearDown(bm::State &state) override
    {
        //
    }

private:
    const camodocal::CataCamera m_camera;
    const double w1, l1, w2, l2, h;
};

static void coc_omni_spaceToImage(bm::State &state)
{
    // clang-format off
    const camodocal::CataCamera camera{"camera",
                                       1280, 800,
                                       0.894975,
                                       -0.344504, 0.0984552,
                                       -0.00403995, 0.00610364,
                                       758.355, 757.615,
                                       646.72, 395.001};
    // clang-format on

    const auto pt = random::pointInFrustum(0.9, 0.6, 1.5, 1, 0.6);
    for ([[maybe_unused]] auto _ : state) {
        Vector2d px;
        camera.spaceToPlane(pt, px);
    }
}

BENCHMARK(coc_omni_spaceToImage);

static void coc_omni_imageToRay(bm::State &state)
{
    // clang-format off
    const camodocal::CataCamera camera{"camera",
                                       1280, 800,
                                       0.894975,
                                       -0.344504, 0.0984552,
                                       -0.00403995, 0.00610364,
                                       758.355, 757.615,
                                       646.72, 395.001};
    // clang-format on

    const auto px = random::pointInImage(640, 480);
    for ([[maybe_unused]] auto _ : state) {
        Vector3d ray;
        camera.liftProjective(px, ray);
    }
}

BENCHMARK(coc_omni_imageToRay);

static void coc_fisheye_spaceToImage(bm::State &state)
{
    // clang-format off
    const camodocal::EquidistantCamera camera{"camera",
                                              640, 480,
                                              -0.01648,  -0.00203, 0.00069, -0.00048,
                                              419.22826, 420.42160,
                                              655.45487, 389.66377};
    // clang-format on

    const Vector3d pt{0.1, 0.2, 1.3};
    for ([[maybe_unused]] auto _ : state) {
        Vector2d px;
        camera.spaceToPlane(pt, px);
    }
}

BENCHMARK(coc_fisheye_spaceToImage);

static void coc_fisheye_spaceToImage_revised(bm::State &state)
{
    // clang-format off
    const camodocal::EquidistantCamera camera{"camera",
                                              640, 480,
                                              -0.01648,  -0.00203, 0.00069, -0.00048,
                                              419.22826, 420.42160,
                                              655.45487, 389.66377};
    // clang-format on

    const auto pt = random::pointInFrustum(0.9, 0.6, 1.5, 1, 0.6);
    for ([[maybe_unused]] auto _ : state) {
        Vector2d px;
        camera.spaceToPlane2(pt, px);
    }
}

BENCHMARK(coc_fisheye_spaceToImage_revised);

static void coc_fisheye_imageToRay(bm::State &state)
{
    // clang-format off
    const camodocal::EquidistantCamera camera{"camera",
                                              640, 480,
                                              -0.01648,  -0.00203, 0.00069, -0.00048,
                                              419.22826, 420.42160,
                                              655.45487, 389.66377};
    // clang-format on

    const auto px = random::pointInImage(640, 480);
    for ([[maybe_unused]] auto _ : state) {
        Vector3d ray;
        camera.liftProjective(px, ray);
    }
}

BENCHMARK(coc_fisheye_imageToRay);

static void coc_fisheye_imageToRay_iteration1(bm::State &state)
{
    // clang-format off
    const camodocal::EquidistantCamera camera{"camera",
                                              640, 480,
                                              -0.01648,  -0.00203, 0.00069, -0.00048,
                                              419.22826, 420.42160,
                                              655.45487, 389.66377};
    // clang-format on

    const auto px = random::pointInImage(640, 480);
    for ([[maybe_unused]] auto _ : state) {
        Vector3d ray;
        camera.liftProjectiveByIteration1(px, ray);
    }
}

BENCHMARK(coc_fisheye_imageToRay_iteration1);

static void coc_fisheye_imageToRay_iteration2(bm::State &state)
{
    // clang-format off
    const camodocal::EquidistantCamera camera{"camera",
                                              640, 480,
                                              -0.01648,  -0.00203, 0.00069, -0.00048,
                                              419.22826, 420.42160,
                                              655.45487, 389.66377};
    // clang-format on

    const auto px = random::pointInImage(640, 480);
    for ([[maybe_unused]] auto _ : state) {
        Vector3d ray;
        camera.liftProjectiveByIteration2(px, ray);
    }
}

BENCHMARK(coc_fisheye_imageToRay_iteration2);

static void calibkit_omni_spaceToImage(bm::State &state)
{
    // Same parameters as CamOdoCal test case
    OmnidirectionalCameraModel camera;
    camera.setFocalLength(758.355);
    camera.setAspectRatio(0.999024);
    camera.setPrincipalPoint(646.72, 395.001);
    camera.setMirrorDistortion(0.894975);
    camera.setTangentialDistortion(-0.00403995, 0.00610364);
    camera.setRadialDistortion(-0.344504, 0.0984552);

    const auto pt = random::pointInFrustum(0.9, 0.6, 1.5, 1, 0.6);
    for ([[maybe_unused]] auto _ : state) {
        [[maybe_unused]] const auto px = camera.spaceToImage(pt);
    }
}

BENCHMARK(calibkit_omni_spaceToImage);

static void calibkit_fisheye_spaceToImage(bm::State &state)
{
    // Same parameters as CamOdoCal test case
    FisheyeCameraModel camera;
    camera.setFocalLength(419.22826);
    camera.setAspectRatio(1.002847);
    camera.setPrincipalPoint(655.45487, 389.66377);
    camera.setRadialDistortion(-0.01648, -0.00203, 0.00069, -0.00048);

    const auto pt = random::pointInFrustum(0.9, 0.6, 1.5, 1, 0.6);
    for ([[maybe_unused]] auto _ : state) {
        [[maybe_unused]] const auto px = camera.spaceToImage(pt);
    }
}

BENCHMARK(calibkit_fisheye_spaceToImage);

static void calibkit_fisheye_imageToRay(bm::State &state)
{
    // Same parameters as CamOdoCal test case
    FisheyeCameraModel camera;
    camera.setFocalLength(419.22826);
    camera.setAspectRatio(1.002847);
    camera.setPrincipalPoint(655.45487, 389.66377);
    camera.setRadialDistortion(-0.01648, -0.00203, 0.00069, -0.00048);

    const auto px = random::pointInImage(640, 480);
    for ([[maybe_unused]] auto _ : state) {
        [[maybe_unused]] const auto ray = camera.imageToSpace(px);
    }
}

BENCHMARK(calibkit_fisheye_imageToRay);
