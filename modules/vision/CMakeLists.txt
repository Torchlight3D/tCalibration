set(IMGPROC_TARGET_NAME Vision)

add_library(${IMGPROC_TARGET_NAME})
add_library(${NS_PREFIX}::${IMGPROC_TARGET_NAME} ALIAS ${IMGPROC_TARGET_NAME})

file(GLOB CALIBTARGET_INTERNAL_HEADERS
    src/*.h
    src/mtf/*.h
)
file(GLOB CALIBTARGET_SOURCES
    src/*.cpp
    src/mtf/*.cc
)

target_sources(${IMGPROC_TARGET_NAME}
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
    core
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

target_include_directories(${IMGPROC_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${IMGPROC_TARGET_NAME}
    PUBLIC
        ${OpenCVLibs}
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
)

install(
    TARGETS ${IMGPROC_TARGET_NAME}
    EXPORT ${IMGPROC_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${IMGPROC_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()
