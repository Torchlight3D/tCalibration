set(CALIBTARGET_TARGET_NAME CalibTarget)

add_library(${CALIBTARGET_TARGET_NAME})
add_library(${NS_PREFIX}::${CALIBTARGET_TARGET_NAME} ALIAS ${CALIBTARGET_TARGET_NAME})

file(GLOB CALIBTARGET_INTERNAL_HEADERS
    src/*.h
    src/apriltag/*.h
    src/apriltag_mit/*.h
)
file(GLOB CALIBTARGET_SOURCES
    src/*.cpp
    src/apriltag/*.cc
    src/apriltag_mit/*.cc
)

target_sources(${CALIBTARGET_TARGET_NAME}
    PRIVATE
        ${CALIBTARGET_INTERNAL_HEADERS}
        ${CALIBTARGET_SOURCES}
)

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

# Eigen, apt default 3.3
set(Eigen3_DIR ${ACTIVE_DEPENDS_PREFIX}/eigen/3.4.0/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)

target_include_directories(${CALIBTARGET_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        ${OpenCV_INCLUDE_DIRS}
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${CALIBTARGET_TARGET_NAME}
    PUBLIC
        ${OpenCVLibs}
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
)

install(
    TARGETS ${CALIBTARGET_TARGET_NAME}
    EXPORT ${CALIBTARGET_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${CALIBTARGET_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()
