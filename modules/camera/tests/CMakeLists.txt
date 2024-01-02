set(CAMERA_TEST_TARGET camera_test)

add_executable(${CAMERA_TEST_TARGET})

target_sources(${CAMERA_TEST_TARGET}
    PRIVATE
        tst_camera.cpp
        tst_divisionundistortion.cpp
        tst_doublesphere.cpp
        tst_extendedunified.cpp
        tst_fisheye.cpp
        tst_fov.cpp
        tst_omnidirectional.cpp
        tst_orthographic.cpp
        tst_pinhole.cpp
)

# googletest, Ubuntu 20.04 default 1.10.0.
set(GTest_DIR ${ACTIVE_DEPENDS_PREFIX}/googletest/1.14.0/lib/cmake/GTest)
find_package(GTest REQUIRED)

# OpenCV, apt default 4.2. TODO: Duplicated code.
if(MSVC)
    set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.5.5x)
elseif()
    set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.5.5x/lib/cmake/opencv4)
endif()
set(OpenCVModules
    calib3d
    core
    imgcodecs
    imgproc
)

foreach(OpenCVModule ${OpenCVModules})
    set(OpenCVLibs ${OpenCVLibs} opencv_${OpenCVModule})
endforeach()

find_package(OpenCV REQUIRED
    COMPONENTS
        ${OpenCVModules}
)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "OpenCV library not found.")
endif()

target_include_directories(${CAMERA_TEST_TARGET}
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${CAMERA_TEST_TARGET}
    PRIVATE
        ${NS_PREFIX}::Camera
        GTest::gtest
        GTest::gtest_main
        ${OpenCVLibs}
)

gtest_discover_tests(${CAMERA_TEST_TARGET}
    DISCOVERY_MODE PRE_TEST
)
