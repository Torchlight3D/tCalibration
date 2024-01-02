# TODO: make a cmake function

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

install(
    TARGETS csv-parser magic_enum json
    EXPORT ExternalConfig
)

install(
    EXPORT ExternalConfig
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/csv-parser/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/magic_enum/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/json/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/extern
)
