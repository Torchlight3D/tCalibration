set(IMU_TARGET_NAME Motion)

add_library(${IMU_TARGET_NAME})
add_library(${NS_PREFIX}::${IMU_TARGET_NAME} ALIAS ${IMU_TARGET_NAME})

# TODO: use src/CMakeLists.txt to control source files
file(GLOB IMU_INTERNAL_HEADERS
    src/type/*.h
    src/type/*.hpp
    src/algo/*.h
    src/*.h
)
file(GLOB IMU_SOURCES
    src/type/*.cpp
    src/algo/*.cpp
)

target_sources(${IMU_TARGET_NAME}
    PRIVATE
        ${IMU_INTERNAL_HEADERS}
        ${IMU_SOURCES}
)

target_include_directories(${IMU_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${IMU_TARGET_NAME}
    PUBLIC   
        ${NS_PREFIX}::Core
        ${NS_PREFIX}::Math
)

install(
    TARGETS ${IMU_TARGET_NAME}
    EXPORT ${IMU_TARGET_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${IMU_TARGET_NAME}Config
    NAMESPACE ${NS_PREFIX}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TEST)
    add_subdirectory(tests)
endif()


# TODO: Help IDE organize files
#source_group(

#)
