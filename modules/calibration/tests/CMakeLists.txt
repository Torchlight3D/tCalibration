set(CALIBRATION_TEST_TARGET calibration_test)

add_executable(${CALIBRATION_TEST_TARGET})

target_sources(${CALIBRATION_TEST_TARGET}
    PRIVATE
        tst_cam_intrinsics.cpp
        tst_ibse.cpp
        tst_io.cpp
        tst_imu_intrinsics.cpp
        tst_spline.cpp
        tst_stereo.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${CALIBRATION_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Calibration
        GTest::gtest
        GTest::gtest_main
        csv-parser
)

gtest_discover_tests(${CALIBRATION_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)
