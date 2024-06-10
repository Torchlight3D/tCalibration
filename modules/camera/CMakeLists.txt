option(USE_CAMODOCAL "Use VINS' camodocal library as algorithm reference." ON)

set(CAMERA_TARGET_NAME Camera)

add_library(${CAMERA_TARGET_NAME})
add_library(${NS_PREFIX}::${CAMERA_TARGET_NAME} ALIAS ${CAMERA_TARGET_NAME})

file(GLOB CAMERA_INTERNAL_HEADERS
    src/geometric/*.h
    src/photometric/*.h
    src/*.h
)
file(GLOB CAMERA_SOURCES
    src/geometric/*.cpp
    src/photometric/*.cpp
    src/*.cpp
)

target_sources(${CAMERA_TARGET_NAME}
    PRIVATE
        ${CAMERA_INTERNAL_HEADERS}
        ${CAMERA_SOURCES}
)

target_include_directories(${CAMERA_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${CAMERA_TARGET_NAME}
    PUBLIC
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
    PRIVATE
        magic_enum
)

if(USE_CAMODOCAL)
    file(GLOB CAMODOCAL_SOURCES
        src/camodocal/*.h
        src/camodocal/*.cpp
    )

    target_sources(${CAMERA_TARGET_NAME}
        PRIVATE
            ${CAMODOCAL_SOURCES}
    )

    # OpenCV, apt default 4.2. TODO: Duplicated code.
    if(MSVC)
        set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.10.0x)
    elseif()
        set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.10.0x/lib/cmake/opencv4)
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

    target_include_directories(${CAMERA_TARGET_NAME}
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(${CAMERA_TARGET_NAME}
        PUBLIC
            ${OpenCVLibs}
    )
endif()

## Install
install(
    TARGETS ${CAMERA_TARGET_NAME}
    EXPORT ${CAMERA_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${CAMERA_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    FILE ${CAMERA_TARGET_NAME}Targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/${CAMERA_TARGET_NAME}Config.cmake.in
    ${CAMERA_TARGET_NAME}Config.cmake 
    @ONLY
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${CAMERA_TARGET_NAME}Config.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)


if(BUILD_TEST)
    add_subdirectory(tests)
endif()
