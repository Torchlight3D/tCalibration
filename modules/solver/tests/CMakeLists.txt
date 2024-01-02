set(SOLVER_TEST_TARGET solver_test)

add_executable(${SOLVER_TEST_TARGET})

target_sources(${SOLVER_TEST_TARGET}
    PRIVATE
        test_utils.h
        tst_brent.cpp
#        main.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${SOLVER_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Solver
        GTest::gtest
        GTest::gtest_main
)

gtest_discover_tests(${SOLVER_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)
