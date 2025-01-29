cmake_minimum_required(VERSION 3.16.3)

project(CalibKit
    VERSION 0.1.0
    DESCRIPTION "Multi purpose calibration toolkit."
    LANGUAGES CXX
)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Custom build options
option(BUILD_SHARED "Build CalibKit dynamically or not" ON)
option(BUILD_APP "Build GUI app depends on CalibKit" OFF)
option(BUILD_TEST "Build unit tests of CalibKit" ON)

# We dont want the users to install CalibKit in any default (root) directories
function(check_install_dir)
    if(CMAKE_INSTALL_PREFIX STREQUAL ""
            OR CMAKE_INSTALL_PREFIX MATCHES "^C:/Program Files"
            OR CMAKE_INSTALL_PREFIX MATCHES "^C:/Program Files (x86)"
            OR CMAKE_INSTALL_PREFIX MATCHES "^/usr/local")
        message(WARNING "Please define a custom install path with 'CMAKE_INSTALL_PREFIX'.")
    endif()
endfunction()

#check_install_dir()

# Common variables
set(NS_PREFIX "tl")

# External dependencies location
set(WIN_DEPENDS_PREFIX "D:/Dependencies")
set(LINUX_DEPENDS_PREFIX $ENV{HOME}/Dependencies)
set(WIN_GIT_PREFIX "D:/Git")
set(LINUX_GIT_PREFIX $ENV{HOME}/Git)

if (MSVC)
    set(ACTIVE_DEPENDS_PREFIX ${WIN_DEPENDS_PREFIX})
    set(ACTIVE_GIT_PREFIX ${WIN_GIT_PREFIX})
else()
    set(ACTIVE_DEPENDS_PREFIX ${LINUX_DEPENDS_PREFIX})
    set(ACTIVE_GIT_PREFIX ${LINUX_GIT_PREFIX})
endif()
message(STATUS "Active dependencies lcoation: ${ACTIVE_DEPENDS_PREFIX}")

if (MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
    add_compile_definitions(NOMINMAX)
    # Generate PDB files under Windows
    add_compile_options("$<$<NOT:$<CONFIG:Debug>>:/Zi>")
    add_link_options("$<$<NOT:$<CONFIG:Debug>>:/DEBUG>")
    add_link_options("$<$<NOT:$<CONFIG:Debug>>:/OPT:REF>")
    add_link_options("$<$<NOT:$<CONFIG:Debug>>:/OPT:ICF>")
endif()

if(BUILD_TEST)
    enable_testing()
endif()

add_subdirectory(extern)

add_subdirectory(core)
add_subdirectory(math)
add_subdirectory(motion)
add_subdirectory(camera)
add_subdirectory(vision)
add_subdirectory(mvs)
add_subdirectory(calibration)
add_subdirectory(vio)
add_subdirectory(device)
add_subdirectory(scene3d)

add_subdirectory(application)

configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    @ONLY
)

install(
    FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
