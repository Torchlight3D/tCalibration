set(MOTION_TARGET_NAME Motion)

add_library(${MOTION_TARGET_NAME})
add_library(${NS_PREFIX}::${MOTION_TARGET_NAME} ALIAS ${MOTION_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB MOTION_HEADERS
    src/algo/*.h
    src/*.h
)
file(GLOB MOTION_SOURCES
    src/algo/*.cpp
    src/*.cpp
)

target_sources(${MOTION_TARGET_NAME}
    PRIVATE
        ${MOTION_HEADERS}
        ${MOTION_SOURCES}
)

target_include_directories(${MOTION_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Sophus
set(Eigen3_DIR ${ACTIVE_DEPENDS_PREFIX}/eigen/3.4.0/share/eigen3/cmake)
set(Sophus_DIR ${ACTIVE_DEPENDS_PREFIX}/sophus/1.x/share/sophus/cmake)
find_package(Sophus REQUIRED)

target_link_libraries(${MOTION_TARGET_NAME}
    PUBLIC   
        ${NS_PREFIX}::Math
        Sophus::Sophus
        nlohmann_json::nlohmann_json
)

install(
    TARGETS ${MOTION_TARGET_NAME}
    EXPORT ${MOTION_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${MOTION_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()


# TODO: Help IDE organize files
#source_group(

#)
