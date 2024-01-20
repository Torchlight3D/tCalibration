set(IMU_TEST_TARGET imu_test)

add_executable(${IMU_TEST_TARGET})

target_sources(${IMU_TEST_TARGET}
    PRIVATE
        tst_sew.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${IMU_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Motion
        GTest::gtest
        GTest::gtest_main
        json
)

gtest_discover_tests(${IMU_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)

# TODO: install sample data to binary directory
