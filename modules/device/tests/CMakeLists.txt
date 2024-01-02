set(DEVICE_TEST_TARGET device_test)

add_executable(${DEVICE_TEST_TARGET})

target_sources(${DEVICE_TEST_TARGET}
    PRIVATE
        tst_recorder.cpp
)

set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

target_link_libraries(${DEVICE_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Device
        GTest::gtest
        GTest::gtest_main
)

gtest_discover_tests(${DEVICE_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)

# TODO: install sample data to binary directory
