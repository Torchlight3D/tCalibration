# The newer ubuntu distros set GCC default link flag -pie (position
# independent executable), which marks e_type as ET_DYN on the binary files.
# Consequently, the OS recognizes them as shared library. To work around it, it
# may be necessary to add -no-pie on compiler option. This only applied on Linux
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU"
        AND UNIX
        AND NOT APPLE
        AND NOT TL_SKIP_PIE_CHANGE)
  message(STATUS "Setting -no-pie on EXE flags; use TL_SKIP_PIE_CHANGE=TRUE to avoid" )
  set(CMAKE_EXE_LINKER_FLAGS "-no-pie")
endif()

add_compile_definitions(
    # No support for Qt5
    QT_DISABLE_DEPRECATED_BEFORE=0x050F00
)

set(QtModules
    Concurrent
    Network
    Svg
    Widgets
)

find_package(Qt6 REQUIRED
    COMPONENTS
        ${QtModules}
        LinguistTools
)

# GUI Module. 
# TODO:
# 1. Make standalone library
set(GUI_MODULE_SRCS
    gui/Animation.h
    gui/Animation.cpp
    gui/AutoDeleteThread.h
    gui/AutoDeleteThread.cpp
    gui/guiutils.h
    gui/guiutils.cpp
    gui/ImageView.h
    gui/ImageView.cpp
    gui/IPv4LineEdit.h
    gui/IPv4LineEdit.cpp
    gui/LedIndicator.h
    gui/LedIndicator.cpp
    gui/LogWidget.h
    gui/LogWidget.cpp
    gui/messagelogwidget.h
    gui/messagelogwidget.cpp
    gui/qcolorutils.h
    gui/qcolorutils.cpp
    gui/qfontutils.h
    gui/qfontutils.cpp
    gui/qimageutils.h
    gui/qimageutils.cpp
    gui/qstringutils.h
    gui/qstringutils.cpp
    gui/qstyleutils.h
    gui/qstyleutils.cpp
    gui/QtOpenCV.h
    gui/QtOpenCV.cpp
    gui/qwidgetutils.h
    gui/qwidgetutils.cpp
    gui/StdOStreamRedirector.h
    gui/WidgetAnimation.h
    gui/WidgetAnimation.cpp
    gui/WidgetAnimationManager.h
    gui/WidgetAnimationManager.cpp
    gui/WidgetAnimator.h
    gui/WidgetAnimator.cpp
)

# Application
set(APP_SRCS
    AppConfigs.h
    AppConfigs.cpp
    CalibBoardConfigsWidget.h
    CalibBoardConfigsWidget.cpp
    CalibBoardConfigsWidget.ui
    EthernetAdaptorHelper.h
    EthernetAdaptorHelper.cpp
    StereoModuleCalibrationConfigWidget.h
    StereoModuleCalibrationConfigWidget.cpp
    StereoModuleCalibrationConfigWidget.ui
    StereoModuleCalibResultView.h
    StereoModuleCalibResultView.cpp
    StereoModuleData.h
    StereoModuleData.cpp
    StereoModuleImageQualityView.h
    StereoModuleImageQualityView.cpp
    StereoModuleTask.h
    StereoModuleTask.cpp
    StereoModuleToolView.h
    StereoModuleToolView.cpp
    StereoModuleVerificationConfigWidget.h
    StereoModuleVerificationConfigWidget.cpp
    StereoModuleVerificationConfigWidget.ui
    StereoModuleVerifyResultView.h
    StereoModuleVerifyResultView.cpp

    main.cpp
)

set(APP_RC
    resources/app.qrc
)

set(APP_WIN_RC
    resources/app_win.rc
)

set(APP_TS_FILES
    locale/CalibrationToolbox_zh_CN.ts
)

# Use qt_standard_project_setup() after Qt 6.3
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# TODO: Not recommended usage
qt_create_translation(APP_QM_FILES ${CMAKE_SOURCE_DIR} ${APP_TS_FILES})

# TODO: How to use qt_add_resources()

qt_add_executable(CalibrationToolbox
    MANUAL_FINALIZATION
    ${GUI_MODULE_SRCS}
    ${APP_SRCS}
    ${APP_RC}
    ${APP_QM_FILES}
    ${APP_WIN_RC}
)

set_target_properties(CalibrationToolbox
    PROPERTIES
        WIN32_EXECUTABLE TRUE
)

target_include_directories(CalibrationToolbox
    PRIVATE
        # FIXME: This is seriously wrong. Workaround to help locating promoted widget header files.
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(CalibrationToolbox
    PRIVATE
        Qt::Concurrent
        Qt::CorePrivate # For QRingBuffer
        Qt::Widgets
        Qt::Network
        Qt::Svg
        csv-parser
        ${NS_PREFIX}::Calibration
        ${NS_PREFIX}::Scene3d
        ${NS_PREFIX}::Device
)

### Install
install(TARGETS CalibrationToolbox
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Translations
install(
    FILES
        ${APP_QM_FILES}
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}/locale
)

# 3rd-party libs
if(MSVC OR MSYS OR MINGW) # Windows compiler
    # Qt libs
    get_target_property(QtCore_LOCATION Qt6::Core LOCATION)
    get_filename_component(QT_DLL_DIR ${QtCore_LOCATION} PATH)

    install(
        FILES
            ${QT_DLL_DIR}/Qt6Core.dll
            ${QT_DLL_DIR}/Qt6Gui.dll
            ${QT_DLL_DIR}/Qt6Widgets.dll
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}
    )

    # Qt platform plugins
    install(
        FILES
            ${QT_DLL_DIR}/../plugins/platforms/qminimal.dll
            ${QT_DLL_DIR}/../plugins/platforms/qwindows.dll
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}/platforms
    )

    # Qt image format plugins
    install(
        FILES
            ${QT_DLL_DIR}/../plugins/imageformats/qgif.dll
            ${QT_DLL_DIR}/../plugins/imageformats/qico.dll
            ${QT_DLL_DIR}/../plugins/imageformats/qjpeg.dll
            ${QT_DLL_DIR}/../plugins/imageformats/qsvg.dll
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}/imageformats
    )

    # Qt icon engine plugin
    install(
        FILES
            ${QT_DLL_DIR}/../plugins/iconengines/qsvgicon.dll
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}/iconengines
    )

    # Qt Styles Plugin
    install(
        FILES
            ${QT_DLL_DIR}/../plugins/styles/qwindowsvistastyle.dll
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}/styles
    )

    # TODO
else() # Unix compiler
    # Qt Stuffs
    get_target_property(QtCore_LOCATION Qt6::Core LOCATION)
    get_filename_component(QT_SHARED_LIB_DIR ${QtCore_LOCATION} PATH)

    message(STATUS "QtCore location: ${QT_SHARED_LIB_DIR}")

    # TODO:
    # 1. Don't hardcode version
    # 2. Use function to generate list
    install(
        FILES
            # required by Qt6Core
            ${QT_SHARED_LIB_DIR}/libicudata.so
            ${QT_SHARED_LIB_DIR}/libicudata.so.56
            ${QT_SHARED_LIB_DIR}/libicudata.so.56.1

            # required by Qt6Core
            ${QT_SHARED_LIB_DIR}/libicui18n.so
            ${QT_SHARED_LIB_DIR}/libicui18n.so.56
            ${QT_SHARED_LIB_DIR}/libicui18n.so.56.1

            # required by Qt6Core
            ${QT_SHARED_LIB_DIR}/libicuuc.so
            ${QT_SHARED_LIB_DIR}/libicuuc.so.56
            ${QT_SHARED_LIB_DIR}/libicuuc.so.56.1

            # Qt6Core, required by Qt6Gui
            ${QT_SHARED_LIB_DIR}/libQt6Core.so
            ${QT_SHARED_LIB_DIR}/libQt6Core.so.6
            ${QT_SHARED_LIB_DIR}/libQt6Core.so.6.2.4

            # Qt6DBus, required by Qt6Gui
            ${QT_SHARED_LIB_DIR}/libQt6DBus.so
            ${QT_SHARED_LIB_DIR}/libQt6DBus.so.6
            ${QT_SHARED_LIB_DIR}/libQt6DBus.so.6.2.4

            # Qt6Gui, required by Qt6Widgets
            ${QT_SHARED_LIB_DIR}/libQt6Gui.so
            ${QT_SHARED_LIB_DIR}/libQt6Gui.so.6
            ${QT_SHARED_LIB_DIR}/libQt6Gui.so.6.2.4

            # Qt6Widgets
            ${QT_SHARED_LIB_DIR}/libQt6Widgets.so
            ${QT_SHARED_LIB_DIR}/libQt6Widgets.so.6
            ${QT_SHARED_LIB_DIR}/libQt6Widgets.so.6.2.4

            # Qt6Network
            ${QT_SHARED_LIB_DIR}/libQt6Network.so
            ${QT_SHARED_LIB_DIR}/libQt6Network.so.6
            ${QT_SHARED_LIB_DIR}/libQt6Network.so.6.2.4

            # More...
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}
    )

    # Qt platform plugins
    install(
        FILES
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqeglfs.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqlinuxfb.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqminimal.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqminimalegl.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqoffscreen.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqvkkhrdisplay.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqvnc.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqwayland-egl.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqwayland-generic.so
            ${QT_SHARED_LIB_DIR}/../plugins/platforms/libqxcb.so
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}/platforms
    )

    # Qt image format plugins
    install(
        FILES
            ${QT_SHARED_LIB_DIR}/../plugins/imageformats/libqgif.so
            ${QT_SHARED_LIB_DIR}/../plugins/imageformats/libqico.so
            ${QT_SHARED_LIB_DIR}/../plugins/imageformats/libqjpeg.so
            ${QT_SHARED_LIB_DIR}/../plugins/imageformats/libqsvg.so
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}/imageformats
    )

    ## Qt XCB GL integrations plugin
    install(
        FILES
            ${QT_SHARED_LIB_DIR}/../plugins/xcbglintegrations/libqxcb-egl-integration.so
            ${QT_SHARED_LIB_DIR}/../plugins/xcbglintegrations/libqxcb-glx-integration.so
        DESTINATION
            ${CMAKE_INSTALL_PREFIX}/xcbglintegrations
    )

endif()

qt_finalize_executable(CalibrationToolbox)

