set(DEVICE_TARGET_NAME Device)

add_library(${DEVICE_TARGET_NAME})
add_library(${NS_PREFIX}::${DEVICE_TARGET_NAME} ALIAS ${DEVICE_TARGET_NAME})

file(GLOB CORE_INTERNAL_HEADERS
    src/io/*.h
    src/*.h
)
file(GLOB CORE_SOURCES
    src/io/*.cpp
    src/*.cpp
)

target_sources(${DEVICE_TARGET_NAME}
    PRIVATE
        ${CORE_INTERNAL_HEADERS}
        ${CORE_SOURCES}
)

# OpenCV, apt default 4.2. TODO: Duplicated code.
if(MSVC)
    set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.10.0x)
elseif()
    set(OpenCV_DIR ${ACTIVE_DEPENDS_PREFIX}/opencv/4.10.0x/lib/cmake/opencv4)
endif()

find_package(OpenCV REQUIRED
    COMPONENTS
        core
        imgcodecs
        imgproc
        videoio
)

target_include_directories(${DEVICE_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${DEVICE_TARGET_NAME}
    PUBLIC
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Motion
        opencv_core
        opencv_imgcodecs
        opencv_imgproc
        opencv_videoio
    PRIVATE
        csv-parser
        nlohmann_json::nlohmann_json
)

install(
    TARGETS ${DEVICE_TARGET_NAME}
    EXPORT ${DEVICE_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${DEVICE_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()
