# Copyright (c) 2013-2017, EPFL/Blue Brain Project
#                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
#                          Nadir Román Guerrero <nadir.romanguerrero@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

# Base configuration
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(Brion VERSION 3.2.0)
set(Brion_VERSION_ABI 10)

# Enforce C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Allow per target specification of build output folder
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Get the last commit hash
execute_process(COMMAND "/usr/bin/git" "rev-parse" "--short=8" "HEAD"
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                RESULT_VARIABLE GIT_CMD_RESULT
                OUTPUT_VARIABLE GIT_CMD_OUTPUT)

# Variables needed to configure the version files
set(BRION_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(BRION_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(BRION_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(BRION_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
string(REPLACE "\n" "" BRION_GIT_HASH ${GIT_CMD_OUTPUT})
set(BRION_GIT_HASH 0x${BRION_GIT_HASH})
set(BRION_REV_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}.${BRION_GIT_HASH})

# Disable libsonata as submodules (when brought by spack)
option(BRION_SKIP_LIBSONATA_SUBMODULE "Skip libsonata module" OFF)
# Wether to build doxygen documentatin or not
option(BRION_BUILD_DOC "Build documentation for Brion" OFF)
# If no cmake install dir specified, initialize to /usr
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT MSVC)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH
    "${PROJECT_NAME} install prefix" FORCE)
endif()

# Include custom cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)
include(CompileOptions)
include(ChoosePython)

# Configure base directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Configure used python version
set(USE_PYTHON_VERSION 3)

# Python wheels needed variables
set(BRION_DESCRIPTION "The Blue Brain C++ I/O library")
set(BRION_MAINTAINER_NAME "Blue Brain Project")
set(BRION_MAINTAINER_EMAIL "<bbp-open-source@googlegroups.com>")
set(BRION_MAINTAINER "${BRION_MAINTAINER_NAME} ${BRION_MAINTAINER_EMAIL}")
set(BRION_LICENSE LGPL)
set(BRION_DEB_DEPENDS libhdf5-serial-dev libboost-date-time-dev
  libboost-filesystem-dev libboost-iostreams-dev libboost-regex-dev
  libboost-system-dev libboost-test-dev libboost-thread-dev
  python3-sphinx python3-lxml python3-dev python3-numpy)
set(BRION_PORT_DEPENDS sphinx py-lxml)

# Required and optional packages
find_package(Boost REQUIRED COMPONENTS
  date_time filesystem iostreams program_options regex system unit_test_framework)
find_package(Boost COMPONENTS python3)
if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    set(Python_ADDITIONAL_VERSIONS 3)
    find_package(PythonInterp)
    find_package(PythonLibs)
    find_package(NumPy 1.7 REQUIRED)
else()
    find_package(Python3 COMPONENTS Interpreter Development NumPy 1.7)
endif()
find_package(glm QUIET)
find_package(HighFive QUIET)
find_package(sonata QUIET)
find_package(MVDTool QUIET)
find_package(OpenMP QUIET)
find_package(Sphinx 1.3 QUIET)
find_package(BBPTestData QUIET)

# Enable the code depending on the test data
if(BBPTestData_FOUND)
    add_definitions(-DBRION_USE_BBPTESTDATA)
endif()

include(CTest)
enable_testing()

# Compile dependencies
add_subdirectory(deps)

# Create the two main libraries
add_subdirectory(brion)
add_subdirectory(brain)

# Tests
# Generate global test targets. Needed even if tests are not built (CI Jobs)
if(NOT TARGET tests)
    add_custom_target(tests)
    set_target_properties(tests PROPERTIES
        EXCLUDE_FROM_DEFAULT_BUILD ON FOLDER ${PROJECT_BINARY_DIR}/tests)
endif()
add_custom_target(Brion-tests)
set_target_properties(Brion-tests PROPERTIES
  EXCLUDE_FROM_DEFAULT_BUILD ON FOLDER ${PROJECT_BINARY_DIR}/tests)
add_dependencies(tests Brion-tests)

if(NOT TARGET smoketests)
    add_custom_target(smoketests)
    set_target_properties(smoketests PROPERTIES
        EXCLUDE_FROM_DEFAULT_BUILD ON FOLDER ${PROJECT_BINARY_DIR}/tests)
endif()
add_custom_target(Brion-smoketests)
set_target_properties(Brion-smoketests PROPERTIES
  EXCLUDE_FROM_DEFAULT_BUILD ON FOLDER ${PROJECT_BINARY_DIR}/tests)
add_dependencies(smoketests Brion-smoketests)

add_subdirectory(tests)

# Utilities
add_subdirectory(apps)

# Generate C++ API Documentation
# General doxygen target to drive all documentation build
if(NOT TARGET doxygen)
        add_custom_target(doxygen)
        set_target_properties(doxygen PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD ON)
endif()
# Dummy target for Jenkins
if(NOT TARGET doxycopy)
    add_custom_target(doxycopy)
    set_target_properties(doxycopy PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD ON)
endif()
add_dependencies(doxycopy doxygen)
if(BRION_BUILD_DOC)
    set(DOXYGEN_MAINPAGE_MD README.md)
    set(DOXYGEN_EXPAND_AS_DEFINED "BRAIN_API BRION_API")
    set(DOXYGEN_EXTRA_INPUT "${PROJECT_SOURCE_DIR}/README.md ${CMAKE_INSTALL_PREFIX}/include/brain")
    set(DOXYGEN_PREDEFINED_MACROS "BRAIN_API:=")
    include(DocBuild)

    # Generate Python API Documentation
    add_subdirectory(doc/python)
endif(BRION_BUILD_DOC)

# Install the export set
install(
  EXPORT Brion-targets
  FILE BrionConfig.cmake
  DESTINATION share/Brion/CMake
)

export(
    EXPORT Brion-targets
    FILE "share/Brion/CMake/BrionConfig.cmake"
)

export(PACKAGE Brion)
export(PACKAGE Brain)

# fill out setup.py metadata for wheel distribution
configure_file(packaging/python_wheels/setup.py.in
               ${PROJECT_BINARY_DIR}/packaging/python_wheels/setup.py)
               
