set(VISION_TARGET_NAME Vision)

add_library(${VISION_TARGET_NAME})
add_library(${NS_PREFIX}::${VISION_TARGET_NAME} ALIAS ${VISION_TARGET_NAME})

file(GLOB CALIBTARGET_INTERNAL_HEADERS
    src/*.h
    src/bow/*.h
    src/mtf/*.h
    src/target/*.h
    src/target/apriltag_mit/*.h
    src/target/codec/*.h

)
file(GLOB CALIBTARGET_SOURCES
    src/*.cpp
    src/bow/*.cpp
    src/mtf/*.cpp
    src/target/*.cpp
    src/target/apriltag_mit/*.cc
)

target_sources(${VISION_TARGET_NAME}
    PRIVATE
        ${CALIBTARGET_INTERNAL_HEADERS}
        ${CALIBTARGET_SOURCES}
)

# OpenCV, apt default 4.2
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

# apriltag-v3
set(apriltag_DIR ${ACTIVE_DEPENDS_PREFIX}/apriltag/3.4.2/share/apriltag/cmake)
find_package(apriltag REQUIRED)

target_include_directories(${VISION_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${VISION_TARGET_NAME}
    PUBLIC
        ${OpenCVLibs}
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
        nlohmann_json::nlohmann_json
    PRIVATE
        apriltag::apriltag
        magic_enum::magic_enum
        mipp
)

install(
    TARGETS ${VISION_TARGET_NAME}
    EXPORT ${VISION_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${VISION_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()
