set(SOLVER_TARGET_NAME Solver)

add_library(${SOLVER_TARGET_NAME})
add_library(${NS_PREFIX}::${SOLVER_TARGET_NAME} ALIAS ${SOLVER_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB SOLVER_INTERNAL_HEADERS
    src/*.h
)
file(GLOB SOLVER_SOURCES
    src/*.cpp
)

target_sources(${SOLVER_TARGET_NAME}
    PRIVATE
        ${SOLVER_INTERNAL_HEADERS}
        ${SOLVER_SOURCES}
)

target_include_directories(${SOLVER_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${SOLVER_TARGET_NAME}
    PUBLIC
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
)

install(
    TARGETS ${SOLVER_TARGET_NAME}
    EXPORT ${SOLVER_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${SOLVER_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()


# TODO: Help IDE organize files
#source_group(

#)
