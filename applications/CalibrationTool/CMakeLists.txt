set(APP_VERSION_MAJOR 2)
set(APP_VERSION_MINOR 0)
set(APP_VERSION_PATCH 1)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h"
    @ONLY
)

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
    Widgets
    Core5Compat
)

find_package(Qt6 REQUIRED
    COMPONENTS
        ${QtModules}
        LinguistTools
)

set(QCoro6_DIR ${ACTIVE_DEPENDS_PREFIX}/qcoro/0.11.0/lib/cmake/QCoro6)
set(QCoro6Coro_DIR ${ACTIVE_DEPENDS_PREFIX}/qcoro/0.11.0/lib/cmake/QCoro6Coro)
find_package(QCoro6 REQUIRED
    COMPONENTS
        Core
        Network
)
qcoro_enable_coroutines()

# cppzmq
set(ZeroMQ_DIR ${ACTIVE_DEPENDS_PREFIX}/ZeroMQ/4.3.5/CMake)
set(cppzmq_DIR ${ACTIVE_DEPENDS_PREFIX}/cppzmq/4.10.0/share/cmake/cppzmq)
find_package(cppzmq REQUIRED)

# Target: Qt GUI Module. 
# TODO:
# 1. Make standalone library
set(GUI_MODULE_SRCS
    gui/qfileutils.h
    gui/qfileutils.cpp
    gui/qimageutils.h
    gui/qimageutils.cpp
    gui/qnetworkutils.h
    gui/qnetworkutils.cpp
    gui/qstringutils.h
    gui/qstringutils.cpp
    gui/qtanimation.h
    gui/qtanimation.cpp
    gui/qtcoreutils.h
    gui/qtcoreutils.cpp
    gui/qtcvutils.h
    gui/qtcvutils.cpp
    gui/qtgridwidget.h
    gui/qtgridwidget.cpp
    gui/qtimageview.h
    gui/qtimageview.cpp
    gui/qtipv4lineedit.h
    gui/qtipv4lineedit.cpp
    gui/qtjsontreemodel.h
    gui/qtjsontreemodel.cpp
    gui/qtledindicator.h
    gui/qtledindicator.cpp
    gui/qtlistwidget.h
    gui/qtlistwidget.cpp
    gui/qtlogview.cpp
    gui/qtlogview.h
    gui/qtswitchbutton.h
    gui/qtswitchbutton.cpp
    gui/qttabwidget.h
    gui/qttabwidget.cpp
    gui/qtthread.h
    gui/qtthread.cpp
    gui/qtwaitingspinner.h
    gui/qtwaitingspinner.cpp
    gui/qwidgetutils.h
    gui/qwidgetutils.cpp
)

# Target: Application
set(APP_SRCS
    msgbus/types.h
    msgbus/types.cpp
    soc/types.h
    soc/utils.h
    soc/utils.cpp
    zeromq/types.h
    zeromq/utils.h
    zeromq/utils.cpp
    networkutils.h
    networkutils.cpp
    CalibTaskPlayer.h
    CalibTaskPlayer.cpp
    DataConsumer.h
    DataConsumer.cpp
    DeviceClient.h
    DeviceClient.cpp
    DeviceClient_p.h   # internal
    DeviceInfo.h
    DeviceInfo.cpp
    DeviceListItem.h
    DeviceListItem.cpp
    DeviceWatchdog.h
    DeviceWatchdog.cpp
    EthernetAdaptorHelper.h
    EthernetAdaptorHelper.cpp
    GlobalConfigs.h
    GlobalConfigs.cpp
    MainWindow.h
    MainWindow.cpp
    MesClient.h
    MesClient.cpp
    MonoCameraCalibrationTask.h
    MonoCameraCalibrationTask.cpp
    MonoCameraCalibrationView.h
    MonoCameraCalibrationView.cpp
    MonoCameraCalibrationView.ui
    MonoCameraClient.h
    MonoCameraClient.cpp
    MonoCameraVerificationView.h
    MonoCameraVerificationView.cpp
    MonoCameraVerificationView.ui
    ProductInfo.h
    ProductInfo.cpp
    ReportSummary.h
    ReportSummary.cpp
    ReportView.h
    ReportView.cpp
    SharedUdpSocket.h
    SharedUdpSocket.cpp
    StereoCameraCalibrationTask.h
    StereoCameraCalibrationTask.cpp
    StereoCameraCalibrationView.h
    StereoCameraCalibrationView.cpp
    StereoCameraCalibrationView.ui
    StereoCameraClient.h
    StereoCameraClient.cpp
    # StereoCameraVerificationTask.h
    # StereoCameraVerificationTask.cpp
    StereoCameraVerificationView.h
    StereoCameraVerificationView.cpp
    StereoCameraVerificationView.ui
    TaskInfo.h
    TaskInfo.cpp
    ToolView.h

    main.cpp
)

set(APP_RC
    resources/app.qrc
)

set(APP_WIN_RC
    resources/app_win.rc
)

qt_standard_project_setup(
    REQUIRES 6.5.0
    SUPPORTS_UP_TO 6.5.0
    # todo: enable after Qt 6.7
    # I18N_TRANSLATED_LANGUAGES zh_CN
    # I18N_SOURCE_LANGUAGE en
)

# TODO: How to use qt_add_resources()
set(CMAKE_AUTORCC ON)

qt_add_executable(CalibrationTool
    WIN32
    MANUAL_FINALIZATION
    ${GUI_MODULE_SRCS}
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
)

target_link_libraries(CalibrationTool
    PRIVATE
        Qt::Concurrent
        Qt::CorePrivate # For QRingBuffer
        Qt::Widgets
        Qt::Network
        Qt::Core5Compat
        QCoro6::Core
        QCoro6::Network
        cppzmq
        csv-parser
        stduuid
        ${NS_PREFIX}::Calibration
        ${NS_PREFIX}::Device
)

qt_add_translations(CalibrationTool
    TS_FILES
        locale/CalibrationTool_zh_CN.ts
    LUPDATE_OPTIONS
        -no-obsolete
        -locations none
    LRELEASE_OPTIONS
        -nounfinished
        -removeidentical
)

add_dependencies(CalibrationTool
    update_translations
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

