set(MVS_TEST_TARGET mvs_test)

add_executable(${MVS_TEST_TARGET})

target_sources(${MVS_TEST_TARGET}
    PRIVATE
        test_utils.h
        tst_landmark.cpp
        tst_scene.cpp
        tst_view.cpp
        tst_dlspnp.cpp
        tst_p3p.cpp
        tst_sqpnp.cpp
        tst_calibabsolutepose.cpp
        tst_uncalibabsolutepose.cpp
        tst_distortionuncalibabsolutepose.cpp
        tst_ba.cpp
#        main.cpp
)

# googletest, Ubuntu 20.04 default 1.10.0
set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)
if(GTest_FOUND)
    message(STATUS "Found googletest (${GTest_VERSION})")
endif()

target_link_libraries(${MVS_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Mvs
        GTest::gtest
        GTest::gtest_main
)

gtest_discover_tests(${MVS_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)
