# Copyright (c) 2013-2017, EPFL/Blue Brain Project
#                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

list(APPEND CPPCHECK_EXTRA_ARGS
  --suppress=*:${CMAKE_CURRENT_SOURCE_DIR}/json.hpp)

# bug while checking json.hpp, --suppress does not remove the error exit code
if(CPPCHECK_VERSION VERSION_LESS 1.78)
  list(APPEND CPPCHECK_EXTRA_ARGS --error-exitcode=0)
endif()

set(BRION_PUBLIC_HEADERS
  blueConfig.h
  circuit.h
  compartmentReport.h
  compartmentReportPlugin.h
  enums.h
  mesh.h
  morphology.h
  morphologyPlugin.h
  morphologyPlugin.ipp
  pluginInitData.h
  simulationConfig.h
  spikeReport.h
  spikeReportPlugin.h
  synapseSummary.h
  synapse.h
  target.h
  types.h
  nodes.h
  nodeGroup.h
  circuitConfig.h
  csvConfig.h
  )

set(BRION_HEADERS
  constants.h
  detail/hdf5Mutex.h
  detail/json.hpp
  detail/mesh.h
  detail/meshBinary.h
  detail/meshHDF5.h
  detail/morphologyHDF5.h
  detail/skipWhiteSpace.h
  detail/utils.h
  detail/utilsHDF5.h
  )

set(BRION_SOURCES
  blueConfig.cpp
  circuit.cpp
  compartmentReport.cpp
  mesh.cpp
  morphology.cpp
  simulationConfig.cpp
  spikeReport.cpp
  synapseSummary.cpp
  synapse.cpp
  target.cpp
  nodes.cpp
  nodeGroup.cpp
  circuitConfig.cpp
  csvConfig.cpp
  detail/utils.cpp
  )

if(MSVC)
  add_definitions(/wd4251) # missing dll-interface for H5::Exception
endif()

add_subdirectory(plugin)

set(_plugin_lib BrionPlugins)

# when linking Brion statically, only visible and used symbols from an
# application or shared library will be linked. Meaning the plugin registerers
# for simulation reports and morphologies are skipped. This code adds the
# linker options to include all symbols from BrionPlugins in the final library.
if(COMMON_LIBRARY_TYPE MATCHES "STATIC" AND
    (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR
      CMAKE_COMPILER_IS_GCC))
  if(APPLE)
    set(_plugin_lib -Wl,-force_load BrionPlugins)
  else()
    # As --whole-archive serves as a switch, it has to be reset for any
    # subsequent library.
    set(_plugin_lib -Wl,--whole-archive BrionPlugins -Wl,--no-whole-archive)
  endif()
endif()

set(BRION_PUBLIC_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set(BRION_LINK_LIBRARIES
  PUBLIC Servus vmmlib
  PRIVATE Lunchbox HighFive ${Boost_FILESYSTEM_LIBRARIES}
          ${Boost_REGEX_LIBRARIES} ${CMAKE_THREADS_LIB_INIT} ${_plugin_lib}
)

common_library(Brion)
