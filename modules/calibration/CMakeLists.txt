set(CALIBRATION_TARGET_NAME Calibration)

add_library(${CALIBRATION_TARGET_NAME})
add_library(${NS_PREFIX}::${CALIBRATION_TARGET_NAME} ALIAS ${CALIBRATION_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB CALIBRATION_INTERNAL_HEADERS
    src/*.h
    src/*.hpp
    src/io/*.h
)
file(GLOB CALIBRATION_SOURCES
    src/*.cpp
    src/io/*.cpp
)

target_sources(${CALIBRATION_TARGET_NAME}
    PRIVATE
        ${CALIBRATION_INTERNAL_HEADERS}
        ${CALIBRATION_SOURCES}
)

# Sophus
set(Eigen3_DIR ${ACTIVE_DEPENDS_PREFIX}/eigen/3.4.0/share/eigen3/cmake)
set(Sophus_DIR ${ACTIVE_DEPENDS_PREFIX}/sophus/1.x/share/sophus/cmake)
find_package(Sophus REQUIRED)

# OpenCV, apt default 4.2. TODO: Duplicated code.
if(MSVC)
    set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.10.0x)
elseif()
    set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.10.0x/lib/cmake/opencv4)
endif()
set(OpenCVModules
    ccalib
)

foreach(OpenCVModule ${OpenCVModules})
    set(OpenCVLibs ${OpenCVLibs} opencv_${OpenCVModule})
endforeach()

find_package(OpenCV REQUIRED
    COMPONENTS
        ${OpenCVModules}
)

# yaml-cpp, apt default 0.6.0.
set(yaml-cpp_DIR ${ACTIVE_DEPENDS_PREFIX}/yaml-cpp/0.8.0/lib/cmake/yaml-cpp)
find_package(yaml-cpp REQUIRED)

target_include_directories(${CALIBRATION_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${CALIBRATION_TARGET_NAME}
    PUBLIC
        ${NS_PREFIX}::Camera
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
        ${NS_PREFIX}::Motion
        ${NS_PREFIX}::Mvs
        ${NS_PREFIX}::Vision
        Sophus::Sophus
        yaml-cpp::yaml-cpp
    PRIVATE
        ${OpenCVLibs}
        magic_enum::magic_enum
        nlohmann_json::nlohmann_json

)

install(
    TARGETS ${CALIBRATION_TARGET_NAME}
    EXPORT ${CALIBRATION_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${CALIBRATION_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()


# TODO: Help IDE organize files
#source_group(

#)
