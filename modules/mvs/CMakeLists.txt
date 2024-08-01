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
    src/epipolar/*.h
    src/graph/*.h
    src/pnp/*.h
    src/pnp/**/*.h
    src/poses/*.h
    src/registration/*.h
    src/residuals/*.h
    src/sfm/*.h
    src/*.h
)
file(GLOB_RECURSE MVS_SOURCES
    src/ba/*.cpp
    src/desc/*.cpp
    src/epipolar/*.cpp
    src/pnp/*.cpp
    src/pnp/**/*.cpp
    src/pnp/**/*.hpp
    src/poses/*.cpp
    src/registration/*.cpp
    src/residuals/*.cpp
    src/sfm/*.cpp
)

target_sources(${MVS_TARGET_NAME}
    PRIVATE
        ${MVS_INTERNAL_HEADERS}
        ${MVS_SOURCES}
)

# Sophus
set(Sophus_DIR ${ACTIVE_DEPENDS_PREFIX}/sophus/1.x/share/sophus/cmake)
find_package(Sophus REQUIRED)

target_include_directories(${MVS_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${MVS_TARGET_NAME}
    PUBLIC
        ${NS_PREFIX}::Camera
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
        ${NS_PREFIX}::Vision
    PRIVATE
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
