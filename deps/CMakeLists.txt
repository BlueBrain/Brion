# Mandatory dependencies not found in the system will be built
# from submodules

include(FetchContent)

# =============================================================================
# GLM
# =============================================================================
if(NOT glm_FOUND AND NOT TARGET glm)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/glm")
        message(STATUS "Downloading GLM from submodules")
        FetchContent_Declare(
            glm
            GIT_REPOSITORY https://github.com/g-truc/glm.git
            GIT_TAG        0.9.9.8
            GIT_SHALLOW ON
            GIT_SUBMODULES_RECURSE OFF
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glm
        )
        FetchContent_Populate(glm)
    endif()
    
    add_library(glm INTERFACE)
    target_include_directories(glm SYSTEM INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/glm>
            $<INSTALL_INTERFACE:include>)
            
    install(TARGETS glm
        EXPORT Brion-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include
    )

    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/glm/glm
            DESTINATION include)
            
endif()

# =============================================================================
# libsonata
# =============================================================================
if(NOT sonata_FOUND AND NOT TARGET sonata::sonata_shared)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libsonata")
        message(STATUS "Downloading libsonata from submodules")
        FetchContent_Declare(
            libsonata
            GIT_REPOSITORY https://github.com/BlueBrain/libsonata.git
            GIT_TAG        v0.1.9
            GIT_SHALLOW OFF # Required for its own CMakeLists.txt not to fail
            GIT_SUBMODULES_RECURSE ON
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libsonata
        )
        FetchContent_Populate(libsonata)
    endif()

    set(EXTLIB_FROM_SUBMODULES ON CACHE BOOL "" FORCE)
    set(SONATA_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(libsonata)
    add_library(sonata::sonata_shared ALIAS sonata_shared)
endif()

# =============================================================================
# HighFive
# =============================================================================
if(NOT HighFive_FOUND AND NOT TARGET HighFive)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/HighFive")
        message(STATUS "Downloading HighFive from submodules")
        FetchContent_Declare(
            HighFive
            GIT_REPOSITORY https://github.com/BlueBrain/HighFive.git
            GIT_TAG        v2.3.1
            GIT_SHALLOW ON
            GIT_SUBMODULES_RECURSE OFF
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/HighFive
        )
        FetchContent_Populate(HighFive)
    endif()
    add_subdirectory(HighFive)
endif()

# =============================================================================
# MorphIO
# =============================================================================
if(NOT MorphIO_FOUND AND NOT TARGET MorphIO::morphio)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/MorphIO")
        message(STATUS "Downloading MorphIO from submodules")
        FetchContent_Declare(
            MorphIO
            GIT_REPOSITORY https://github.com/BlueBrain/MorphIO.git
            GIT_TAG        v3.3.2
            GIT_SHALLOW ON
            GIT_SUBMODULES_RECURSE ON
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/MorphIO
        )
        FetchContent_Populate(MorphIO)
    endif()

    set(BUILD_BINDINGS OFF CACHE BOOL "" FORCE)
    set(MORPHIO_TESTS OFF CACHE BOOL "" FORCE)

    set(MORPHIO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/MorphIO/include/)

    # Deps
    add_subdirectory(MorphIO/3rdparty/GSL_LITE)
    target_include_directories(gsl-lite SYSTEM INTERFACE)
    add_library(lexertl INTERFACE)
    target_include_directories(lexertl SYSTEM INTERFACE MorphIO/3rdparty/lexertl14/include)

    # Retrieve version
    set(MORPHIO_VERSION_STRING "3.3.2")
    set(MorphIO_VERSION_MAJOR 3)
    set(MorphIO_VERSION_MINOR 3)
    set(MorphIO_VERSION_PATCH 2)
    # Morphio
    add_subdirectory(MorphIO/src)
    add_library(morphiointer INTERFACE)
    target_include_directories(morphiointer INTERFACE ${CMAKE_CURRENT_LIST_DIR}/MorphIO/include)
    target_link_libraries(morphiointer INTERFACE morphio_shared)
    add_library(MorphIO::morphio ALIAS morphiointer)
endif()

# =============================================================================
# MVDTool
# =============================================================================
if(NOT MVDTool_FOUND AND NOT TARGET MVDTool)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/MVDTool")
        message(STATUS "Downloading MVDTool from submodules")
        FetchContent_Declare(
            MVDTool
            GIT_REPOSITORY https://github.com/BlueBrain/MVDTool.git
            GIT_TAG        v2.4.2
            GIT_SHALLOW ON
            #GIT_SUBMODULES # Empty string from 3.16 onwards will not clone submodules
            GIT_SUBMODULES_RECURSE OFF
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/MVDTool
        )
        FetchContent_Populate(MVDTool)
    endif()

    add_library(MVDTool INTERFACE)

    target_include_directories(MVDTool SYSTEM INTERFACE
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MVDTool/include>
          $<INSTALL_INTERFACE:include>)
    target_include_directories(MVDTool SYSTEM INTERFACE ${Boost_INCLUDE_DIR})
    target_link_libraries(MVDTool INTERFACE HighFive)
    target_compile_definitions(MVDTool INTERFACE -DH5_USE_BOOST)
endif()
