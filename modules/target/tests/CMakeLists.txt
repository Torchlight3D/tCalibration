set(CALIBTARGET_TEST_TARGET calibtarget_test)

add_executable(${CALIBTARGET_TEST_TARGET})

target_sources(${CALIBTARGET_TEST_TARGET}
    PRIVATE
        tst_apriltag.cpp
        tst_caltag.cpp
        tst_checkergrid.cpp
        tst_circlegrid.cpp
        tst_multiboard.cpp
        # main.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${CALIBTARGET_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Target
        GTest::gtest
        GTest::gtest_main
)

gtest_discover_tests(${CALIBTARGET_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)
