set(CAMERA_BENCHMARK_TARGET camera_perf)

add_executable(${CAMERA_BENCHMARK_TARGET})

target_sources(${CAMERA_BENCHMARK_TARGET}
    PRIVATE
        perf_camera.cpp
)

set(benchmark_DIR ${ACTIVE_DEPENDS_PREFIX}/benchmark/1.8.3/lib/cmake/benchmark)
find_package(benchmark REQUIRED)

target_link_libraries(${CAMERA_BENCHMARK_TARGET}
    PRIVATE
        benchmark::benchmark
        benchmark::benchmark_main
        ${NS_PREFIX}::Camera
)
