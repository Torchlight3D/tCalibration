# TODO: make a cmake function

# argparse
add_library(argparse INTERFACE)

target_include_directories(argparse
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(argparse
    INTERFACE
        cxx_std_17
)

# csv-parser
add_library(csv-parser INTERFACE)

target_include_directories(csv-parser 
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(csv-parser 
    INTERFACE
        cxx_std_17
)

# graaf
add_library(graaf INTERFACE)

target_include_directories(graaf
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(graaf
    INTERFACE
        cxx_std_20
)

# json
add_library(json INTERFACE)

target_include_directories(json
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(json
    INTERFACE
        cxx_std_17
)

# magic_enum
add_library(magic_enum INTERFACE)

target_include_directories(magic_enum 
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(magic_enum 
    INTERFACE
        cxx_std_17
)

# nanoflann
add_library(nanoflann INTERFACE)

target_include_directories(nanoflann 
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(nanoflann 
    INTERFACE
        cxx_std_17
)

# stduuid
add_library(stduuid INTERFACE)

target_include_directories(stduuid
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(stduuid
    INTERFACE
        cxx_std_20
)

# tabulate
add_library(tabulate INTERFACE)

target_include_directories(tabulate
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(tabulate
    INTERFACE
        cxx_std_17
)

## Install
install(
    TARGETS
        argparse
        csv-parser
        graaf
        json
        magic_enum
        nanoflann
        stduuid
        tabulate
    EXPORT ExternalConfig
)

install(
    EXPORT ExternalConfig
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/argparse/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/csv-parser/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/graaflib/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/json/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/magic_enum/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanoflann/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/stduuid/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/tabulate/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)


