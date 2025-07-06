set(VIO_TARGET_NAME Vio)

add_library(${VIO_TARGET_NAME})
add_library(${NS_PREFIX}::${VIO_TARGET_NAME} ALIAS ${VIO_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB VIO_INTERNAL_HEADERS
    src/factors/*.h
)
file(GLOB VIO_SOURCES
    src/factors/*.cpp
)

target_sources(${VIO_TARGET_NAME}
    PRIVATE
        ${VIO_INTERNAL_HEADERS}
        ${VIO_SOURCES}
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

target_include_directories(${VIO_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${VIO_TARGET_NAME}
    PUBLIC
        ${NS_PREFIX}::Camera
        ${NS_PREFIX}::Math
        ${NS_PREFIX}::Motion
        ${NS_PREFIX}::Vision
        Sophus::Sophus
        yaml-cpp::yaml-cpp
    PRIVATE
        ${OpenCVLibs}
        magic_enum
        nlohmann_json::nlohmann_json
)

install(
    TARGETS ${VIO_TARGET_NAME}
    EXPORT ${VIO_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${VIO_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# if(BUILD_TEST)
#     add_subdirectory(tests)
# endif()


# TODO: Help IDE organize files
#source_group(

#)
