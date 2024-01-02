# Qt
if (NOT MSVC)
    # Workaround for a Qt bug, see
    # https://bugreports.qt.io/browse/QTBUG-97615 for more details
    list(APPEND CMAKE_PREFIX_PATH $ENV{HOME}/Qt/6.2.4/gcc_64)
endif()

set(QtModules
    Core
    OpenGL
    OpenGLWidgets
    Widgets
    Xml
)
find_package(QT NAMES Qt6 Qt5 REQUIRED
    COMPONENTS
        ${QtModules}
)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED
    COMPONENTS
        ${QtModules}
)

foreach(QtModule ${QtModules})
    set(QtLibs ${QtLibs} Qt${QT_VERSION_MAJOR}::${QtModule})
endforeach()


if(${QT_VERSION_MAJOR} GREATER_EQUAL 6
        AND ${QT_VERSION_MINOR} GREATER_EQUAL 3)
    qt_standard_project_setup()
else()
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
endif()

find_package(OpenGL REQUIRED)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SCENE_TARGET_NAME Scene3D)

add_library(${SCENE_TARGET_NAME})
add_library(${NS_PREFIX}::${SCENE_TARGET_NAME} ALIAS ${SCENE_TARGET_NAME})

file(GLOB SCENE_INTERNAL_HEADERS
    src/*.h
    src/vector_render/*.h
)
file(GLOB SCENE_SOURCES
    src/*.cpp
    src/vector_render/*.cpp
)

target_sources(${SCENE_TARGET_NAME}
    PRIVATE
        ${SCENE_INTERNAL_HEADERS}
        ${SCENE_SOURCES}
)

target_include_directories(${SCENE_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${SCENE_TARGET_NAME}
    PUBLIC
        ${QtLibs}
    PRIVATE
        OpenGL::GL
        OpenGL::GLU
)

install(
    TARGETS ${SCENE_TARGET_NAME}
    EXPORT ${SCENE_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${SCENE_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# TODO: Help IDE organize files
#source_group(

#)
