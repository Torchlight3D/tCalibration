set(MATH_TEST_TARGET math_test)

add_executable(${MATH_TEST_TARGET})

target_sources(${MATH_TEST_TARGET}
    PRIVATE
        test_utils.h
        tst_brent.cpp
        tst_eigen.cpp
        tst_kalman.cpp

        # RANSAC
        tst_exhaustiveransac.cpp
        tst_lmed.cpp
        tst_prosac.cpp
        tst_ransac.cpp

        # main.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${MATH_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
        GTest::gtest
        GTest::gtest_main
        glog::glog
)

gtest_discover_tests(${MATH_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)
