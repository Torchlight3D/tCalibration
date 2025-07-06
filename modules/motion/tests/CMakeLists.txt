set(IMU_TEST_TARGET motion_test)

add_executable(${IMU_TEST_TARGET})

target_sources(${IMU_TEST_TARGET}
    PRIVATE
        test_utils.h
        
        tst_imudata.cpp
        tst_imuintrinsics.cpp
        tst_sew.cpp
        tst_spline.cpp
        tst_splinebase.cpp
        tst_splinese3.cpp

        # main.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${IMU_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Motion
        GTest::gtest
        GTest::gtest_main
        nlohmann_json::nlohmann_json
)

gtest_discover_tests(${IMU_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)

# TODO: install sample data to binary directory
