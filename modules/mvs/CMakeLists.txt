set(MVS_TARGET_NAME Mvs)

add_library(${MVS_TARGET_NAME})
add_library(${NS_PREFIX}::${MVS_TARGET_NAME} ALIAS ${MVS_TARGET_NAME})

# TODO:
# 1. seperate the impl folders
# 2. use src/CMakeLists.txt to control source files
# 3. explicitly list source files
file(GLOB_RECURSE MVS_INTERNAL_HEADERS
    src/ba/*.h
    src/desc/*.h
    src/pnp/*.h
    src/residuals/*.h
    src/*.h
)
file(GLOB_RECURSE MVS_SOURCES
    src/ba/*.cpp
    src/desc/*.cpp
    src/pnp/*.cpp
    src/pnp/**/*.cpp
    src/residuals/*.cpp
)

target_sources(${MVS_TARGET_NAME}
    PRIVATE
        ${MVS_INTERNAL_HEADERS}
        ${MVS_SOURCES}
)

# fmt, now used by Sophus, explicitly enable before Cpp20
set(fmt_DIR ${ACTIVE_DEPENDS_PREFIX}/fmt/10.2.1/lib/cmake/fmt)
find_package(fmt REQUIRED)

# Sophus
set(Eigen3_DIR ${ACTIVE_DEPENDS_PREFIX}/eigen/3.4.0/share/eigen3/cmake)
set(Sophus_DIR ${ACTIVE_DEPENDS_PREFIX}/sophus/1.22.10/share/sophus/cmake)
find_package(Sophus REQUIRED)

target_include_directories(${MVS_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${MVS_TARGET_NAME}
    PUBLIC
        ${NS_PREFIX}::Camera
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
        ${NS_PREFIX}::Vision
    PRIVATE
        fmt::fmt
        magic_enum
        Sophus::Sophus
)

install(
    TARGETS ${MVS_TARGET_NAME}
    EXPORT ${MVS_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${MVS_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()

# TODO: Help IDE organize files
#source_group(

#)
