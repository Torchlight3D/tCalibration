set(MVS_TEST_TARGET mvs_test)

add_executable(${MVS_TEST_TARGET})

target_sources(${MVS_TEST_TARGET}
    PRIVATE
        test_utils.h
        test_utils.cpp
        # Description
        tst_landmark.cpp
        tst_scene.cpp
        tst_view.cpp
        tst_viewgraph.cpp
        # PnP
        pnp/tst_dlspnp.cpp
        pnp/tst_mlpnp.cpp
        pnp/tst_p3p.cpp
        pnp/tst_p4pfocal.cpp
        pnp/tst_p4pfocalrd.cpp
        pnp/tst_p5pfocalrd.cpp
        pnp/tst_sqpnp.cpp
        pnp/tst_upnp.cpp
        tst_calibabsolutepose.cpp
        tst_uncalibabsolutepose.cpp
        tst_distortionuncalibabsolutepose.cpp
        tst_estimateabsolutepose.cpp
        # Epipolar
        epipolar/tst_epipolar.cpp
        epipolar/tst_findessential.cpp
        epipolar/tst_findfundamental.cpp
        epipolar/tst_findhomography.cpp
        epipolar/tst_triangulation.cpp
        # BA
        tst_ba.cpp
        # Global Pose
        tst_estimaterotationhybrid.cpp
        tst_estimaterotationlagrange.cpp
        tst_estimaterotationlinear.cpp
        tst_estimaterotationnonlinear.cpp
        tst_estimaterotationrobust.cpp
        tst_estimatepositionligt.cpp
        tst_estimatepositionlinear.cpp
        tst_estimatepositionlud.cpp
        tst_estimatepositionnonlinear.cpp
        tst_alignpotincloud.cpp
        tst_alignrotation.cpp
        tst_optimizerelativeposition.cpp
        # Reconstruction
        tst_selectviewpairs.cpp
        tst_trackbuilder.cpp
        # Graph
        tst_graph.cpp
        # main.cpp
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
