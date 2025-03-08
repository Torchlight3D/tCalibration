set(CORE_TEST_TARGET core_test)

add_executable(${CORE_TEST_TARGET})

target_sources(${CORE_TEST_TARGET}
    PRIVATE
        tst_bimap.cpp
        tst_channel.cpp
        tst_container.cpp
        tst_registerer.cpp
        tst_string.cpp
        tst_templatefactory.cpp
        tst_threadpool.cpp
        # main.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${CORE_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Core
        GTest::gtest
        GTest::gtest_main
        # GTest::gmock
        # GTest::gmock_main
)

gtest_discover_tests(${CORE_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)
