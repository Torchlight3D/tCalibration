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

set(SKIP_EXAMPLES ON CACHE BOOL "Disable Graaf examples" FORCE)
set(SKIP_TESTS ON CACHE BOOL "Disable Graaf tests" FORCE)
set(SKIP_BENCHMARKS ON CACHE BOOL "Disable Graaf benchmark" FORCE)
add_subdirectory(graaf EXCLUDE_FROM_ALL)

add_subdirectory(json EXCLUDE_FROM_ALL)

add_subdirectory(magic_enum EXCLUDE_FROM_ALL)

target_compile_features(magic_enum 
    INTERFACE
        cxx_std_17
)

# MIPP
add_library(mipp INTERFACE)
add_library(MIPP::mipp ALIAS mipp)

target_include_directories(mipp 
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/extern>
)

target_compile_features(mipp 
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
        mipp
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
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/mipp/
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

install(
    TARGETS
        Graaf
        magic_enum
        nlohmann_json
    EXPORT ExternTargets
)
