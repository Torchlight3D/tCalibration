set(CORE_TARGET_NAME Core)

add_library(${CORE_TARGET_NAME})
add_library(${NS_PREFIX}::${CORE_TARGET_NAME} ALIAS ${CORE_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB CORE_INTERNAL_HEADERS
    src/*.h
    src/*.hpp
)
file(GLOB CORE_SOURCES
    src/*.cpp
)

target_sources(${CORE_TARGET_NAME}
    PRIVATE
        ${CORE_INTERNAL_HEADERS}
        ${CORE_SOURCES}
)

# glog, apt default 0.4.0
set(gflags_DIR ${ACTIVE_DEPENDS_PREFIX}/gflags/2.2.2/lib/cmake/gflags)
set(glog_DIR ${ACTIVE_DEPENDS_PREFIX}/glog/0.6.0/lib/cmake/glog)
find_package(glog REQUIRED)
if(glog_FOUND)
    message(STATUS "Found glog (${glog_VERSION})")
endif()

target_include_directories(${CORE_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if (MSVC)
    include(cmake/FindDbgHelp.cmake)

    target_link_libraries(${CORE_TARGET_NAME}
        PUBLIC
            glog::glog
        PRIVATE
            ${DBGHELP_LIBRARIES}
    )
elseif()
    target_link_libraries(${CORE_TARGET_NAME}
        PUBLIC
            glog::glog
    )
endif()

install(
    TARGETS ${CORE_TARGET_NAME}
    EXPORT ${CORE_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${CORE_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()
