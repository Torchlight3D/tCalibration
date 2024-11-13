set(IMU_TARGET_NAME Motion)

add_library(${IMU_TARGET_NAME})
add_library(${NS_PREFIX}::${IMU_TARGET_NAME} ALIAS ${IMU_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB IMU_INTERNAL_HEADERS
    src/algo/*.h
    src/*.h
)
file(GLOB IMU_SOURCES
    src/algo/*.cpp
    src/*.cpp
)

target_sources(${IMU_TARGET_NAME}
    PRIVATE
        ${IMU_INTERNAL_HEADERS}
        ${IMU_SOURCES}
)

target_include_directories(${IMU_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Sophus
set(Sophus_DIR ${ACTIVE_DEPENDS_PREFIX}/sophus/1.x/share/sophus/cmake)
find_package(Sophus REQUIRED)

target_link_libraries(${IMU_TARGET_NAME}
    PUBLIC   
        ${NS_PREFIX}::Math
        Sophus::Sophus
        json
)

install(
    TARGETS ${IMU_TARGET_NAME}
    EXPORT ${IMU_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${IMU_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()


# TODO: Help IDE organize files
#source_group(

#)
