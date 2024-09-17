set(IMGPROC_TEST_TARGET vision_test)

add_executable(${IMGPROC_TEST_TARGET})

target_sources(${IMGPROC_TEST_TARGET}
    PRIVATE
        tst_distortion.cpp
        # Target
        tst_apriltag.cpp
        # tst_caltag.cpp
        tst_checkergrid.cpp
        tst_circlegrid.cpp
        # tst_multiboard.cpp
        # tst_runetag.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${IMGPROC_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Vision
        GTest::gtest
        GTest::gtest_main
)

gtest_discover_tests(${IMGPROC_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)

# TODO: install sample data to binary directory
