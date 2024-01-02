# The newer ubuntu distros set GCC default link flag -pie (position
# independent executable), which marks e_type as ET_DYN on the binary files.
# Consequently, the OS recognizes them as shared library. To work around it, it
# may be necessary to add -no-pie on compiler option. This only applied on Linux
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU"
        AND UNIX
        AND NOT APPLE
        AND NOT AX_SKIP_PIE_CHANGE)
  message(STATUS "Setting -no-pie on EXE flags; use AX_SKIP_PIE_CHANGE=TRUE to avoid" )
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

# gRPC. FIXME: Some of the gRPC dependencies need to be explicitly provided.
set(gRPC_LOCATION ${ACTIVE_DEPENDS_PREFIX}/grpc/1.57.0)
set(absl_DIR ${gRPC_LOCATION}/lib/cmake/absl)
find_package(absl CONFIG REQUIRED)
set(utf8_range_DIR ${gRPC_LOCATION}/lib/cmake/utf8_range)
find_package(utf8_range CONFIG REQUIRED)
set(protobuf_MODULE_COMPATIBLE TRUE)
if(MSVC)
    set(Protobuf_DIR ${gRPC_LOCATION}/cmake)
elseif()
    set(Protobuf_DIR ${gRPC_LOCATION}/lib/cmake/protobuf)
endif()
find_package(Protobuf CONFIG REQUIRED)
set(gRPC_DIR ${gRPC_LOCATION}/lib/cmake/grpc)
find_package(gRPC CONFIG REQUIRED)

# Target 1. Protobuf
add_subdirectory(protos)

# Target 2. GUI Module. 
# TODO:
# 1. Make standalone library
set(GUI_MODULE_SRCS
    gui/AbstractItemListWidget.h
    gui/AbstractItemListWidget.cpp
    gui/Animation.h
    gui/Animation.cpp
    gui/AutoDeleteThread.h
    gui/AutoDeleteThread.cpp
    gui/blurutils.h
    gui/blurutils.cpp
    gui/ColorButton.h
    gui/ColorButton.cpp
    gui/ComboBoxDelegate.h
    gui/ComboBoxDelegate.cpp
    gui/EventFilters.h
    gui/EventFilters.cpp
    gui/guiutils.h
    gui/guiutils.cpp
    gui/ImageView.h
    gui/ImageView.cpp
    gui/IPv4LineEdit.h
    gui/IPv4LineEdit.cpp
    gui/LedIndicator.h
    gui/LedIndicator.cpp
    gui/LineEdit.h
    gui/LineEdit.cpp
    gui/LogWidget.h
    gui/LogWidget.cpp
    gui/NavigationBar.h
    gui/NavigationBar.cpp
    gui/primitiveutils.h
    gui/primitiveutils.cpp
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
    gui/messagelogwidget.h
    gui/messagelogwidget.cpp
    gui/QtOpenCV.h
    gui/QtOpenCV.cpp
    gui/qwidgetutils.h
    gui/qwidgetutils.cpp
    gui/CornerRadius.h
    gui/CornerRadius.cpp
    gui/RoundedFocusFrame.h
    gui/RoundedFocusFrame.cpp
    gui/StatusBadge.h
    gui/StatusBadge.cpp
    gui/StdOStreamRedirector.h
    gui/Switch.h
    gui/Switch.cpp
    gui/stylestateutils.h
    gui/stylestateutils.cpp
    gui/SuspendableWorker.h
    gui/SuspendableWorker.cpp
    gui/TextLabel.h
    gui/TextLabel.cpp
    gui/TheStyle.h
    gui/TheStyle.cpp
    gui/TheStyleOptions.h
    gui/TheStyleOptions.cpp
    gui/TheStyleTypes.h
    gui/TheTheme.h
    gui/TheTheme.cpp
    gui/TreeView.h
    gui/TreeView.cpp
    gui/WidgetAnimation.h
    gui/WidgetAnimation.cpp
    gui/WidgetAnimationManager.h
    gui/WidgetAnimationManager.cpp
    gui/WidgetAnimator.h
    gui/WidgetAnimator.cpp
)

set(GUI_MODULE_RC
    gui/resources/qlementine.qrc
    gui/resources/qlementine_font_roboto.qrc
    $<$<NOT:$<PLATFORM_ID:Windows>>:gui/resources/qlementine_font_inter.qrc>
    $<$<PLATFORM_ID:Windows>:gui/resources/qlementine_font_inter_windows.qrc>
)

# Target 3. Application
set(APP_SRCS
    AppConfigs.h
    AppConfigs.cpp
    CalibBoardConfigsWidget.h
    CalibBoardConfigsWidget.cpp
    CalibBoardConfigsWidget.ui
    EthernetAdaptorHelper.h
    EthernetAdaptorHelper.cpp
    LedIndicatorGrid.h
    LedIndicatorGrid.cpp
    Scene3DViewer.h
    Scene3DViewer.cpp
    StereoModuleCalibrationConfigWidget.h
    StereoModuleCalibrationConfigWidget.cpp
    StereoModuleCalibrationConfigWidget.ui
    StereoModuleCalibrationView.h
    StereoModuleCalibrationView.cpp
    StereoModuleCalibrationView.ui
    StereoModuleCalibResultView.h
    StereoModuleCalibResultView.cpp
    StereoModuleData.h
    StereoModuleData.cpp
    StereoModuleHubClient.h
    StereoModuleHubClient.cpp
    StereoModuleImageQualityView.h
    StereoModuleImageQualityView.cpp
    StereoModuleTask.h
    StereoModuleTask.cpp
    StereoModuleToolView.h
    StereoModuleToolView.cpp
    StereoModuleVerificationConfigWidget.h
    StereoModuleVerificationConfigWidget.cpp
    StereoModuleVerificationConfigWidget.ui
    StereoModuleVerificationView.h
    StereoModuleVerificationView.cpp
    StereoModuleVerificationView.ui
    StereoModuleVerifyResultView.h
    StereoModuleVerifyResultView.cpp
    TestStereoModuleWidget.h
    TestStereoModuleWidget.cpp
    TestStereoModuleWidget.ui
    WidgetGridWidget.h
    WidgetGridWidget.cpp

    main.cpp
)

set(APP_RC
    resources/app.qrc
)

set(APP_WIN_RC
    resources/app_win.rc
)

set(APP_TS_FILES
    locale/CalibrationTool_zh_CN.ts
)

# Use qt_standard_project_setup() after Qt 6.3
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# TODO: Not recommended usage
qt_create_translation(APP_QM_FILES ${CMAKE_SOURCE_DIR} ${APP_TS_FILES})

# TODO: How to use qt_add_resources()

qt_add_executable(CalibrationTool
    MANUAL_FINALIZATION
    ${GUI_MODULE_SRCS}
    ${GUI_MODULE_RC}
    ${APP_SRCS}
    ${APP_RC}
    ${APP_QM_FILES}
    ${APP_WIN_RC}
)

set_target_properties(CalibrationTool
    PROPERTIES
        WIN32_EXECUTABLE TRUE
)

target_include_directories(CalibrationTool
    PRIVATE
        ${CMAKE_SOURCE_DIR} # FIXME: This is seriously wrong. Workaround to
                            # help locating promoted widget header files.
        ${CMAKE_CURRENT_BINARY_DIR}/../proto
)

target_link_libraries(CalibrationTool
    PRIVATE
        Qt::Concurrent
        Qt::CorePrivate # For QRingBuffer
        Qt::Widgets
        Qt::Network
        Qt::Svg
        csv-parser
        gRPC::grpc++
        grpc_proto
        ${_REFLECTION}
        ${NS_PREFIX}::Calibration
        ${NS_PREFIX}::Scene3D
        ${NS_PREFIX}::Device
)

### Install
install(TARGETS CalibrationTool
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

qt_finalize_executable(CalibrationTool)

