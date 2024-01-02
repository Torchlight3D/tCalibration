set(MATH_TARGET_NAME Math)

add_library(${MATH_TARGET_NAME})
add_library(${NS_PREFIX}::${MATH_TARGET_NAME} ALIAS ${MATH_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB MATH_INTERNAL_HEADERS
    src/*.h
    src/ransac/*.h
    src/fft/*.h
)
file(GLOB MATH_SOURCES
    src/*.cpp
    src/ransac/*.cpp
    src/fft/*.cpp
)

# Eigen, apt default 3.3
set(Eigen3_DIR ${ACTIVE_DEPENDS_PREFIX}/eigen/3.4.0/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)

# FFTW, apt default 3.3.8, but failed to locate. Use custom built here
set(FFTW3_DIR ${ACTIVE_DEPENDS_PREFIX}/fftw/3.3.10/lib/cmake/fftw3)
find_package(FFTW3 REQUIRED)
if (FFTW3_FOUND)
    message(STATUS "Found FFTW3 (${FFTW3_VERSION})")
else()
    message(FATAL_ERROR "FFTW3 not found.")
endif()

# glog, apt default 0.4.0
set(gflags_DIR ${ACTIVE_DEPENDS_PREFIX}/gflags/2.2.2/lib/cmake/gflags)
set(glog_DIR ${ACTIVE_DEPENDS_PREFIX}/glog/0.6.0/lib/cmake/glog)
find_package(glog REQUIRED)

# ceres-solver
set(Ceres_DIR ${ACTIVE_DEPENDS_PREFIX}/ceres-solver/2.1.0/lib/cmake/Ceres)
find_package(Ceres REQUIRED)

target_sources(${MATH_TARGET_NAME}
    PRIVATE
        ${MATH_INTERNAL_HEADERS}
        ${MATH_SOURCES}
)

target_include_directories(${MATH_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
        ${FFTW3_INCLUDE_DIRS}
)

target_link_directories(${MATH_TARGET_NAME}
    PRIVATE
        ${FFTW3_LIBRARY_DIRS}
)

target_link_libraries(${MATH_TARGET_NAME}
    PUBLIC
        Ceres::ceres
        ${NS_PREFIX}::Core
    PRIVATE
        FFTW3::fftw3
)

install(
    TARGETS ${MATH_TARGET_NAME}
    EXPORT ${MATH_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${MATH_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()


# TODO: Help IDE organize files
#source_group(

#)
