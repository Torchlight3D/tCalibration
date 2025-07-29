set(SPACE_TARGET_NAME Space)

add_library(${SPACE_TARGET_NAME})
add_library(${NS_PREFIX}::${SPACE_TARGET_NAME} ALIAS ${SPACE_TARGET_NAME})

set(SPACE_HEADERS
    src/ikdtree.h
    src/ioctree.h

)
set(SPACE_SOURCES
    src/ikdtree.cpp
    src/ioctree.cpp
)

target_sources(${SPACE_TARGET_NAME}
    PRIVATE
        ${SPACE_HEADERS}
        ${SPACE_SOURCES}
)

# PCL
set(PCL_DIR ${ACTIVE_DEPENDS_PREFIX}/pcl/1.15.x/cmake)
# todo: remove later, pcl fix this
set(Boost_ROOT ${ACTIVE_DEPENDS_PREFIX}/pcl/1.15.x/3rdParty/Boost)
find_package(PCL REQUIRED
    COMPONENTS
        io
        common
)

target_include_directories(${SPACE_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${SPACE_TARGET_NAME}
    PUBLIC
        ${PCL_LIBRARIES}
        ${NS_PREFIX}::Math
    PRIVATE
)

install(
    TARGETS ${SPACE_TARGET_NAME}
    EXPORT ${SPACE_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${SPACE_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    # add_subdirectory(tests)
endif()
