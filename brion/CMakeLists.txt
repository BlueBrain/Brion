# Copyright (c) 2013-2017, EPFL/Blue Brain Project
# Daniel Nachbaur <daniel.nachbaur@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

list(APPEND CPPCHECK_EXTRA_ARGS
  --suppress=*:${CMAKE_CURRENT_SOURCE_DIR}/json.hpp)

# bug while checking json.hpp, --suppress does not remove the error exit code
if(CPPCHECK_VERSION VERSION_LESS 1.78)
  list(APPEND CPPCHECK_EXTRA_ARGS --error-exitcode=0)
endif()

configure_file(version.h.in
  ${CMAKE_BINARY_DIR}/include/brion/version.h)

set(BRION_PUBLIC_HEADERS
  api.h
  blueConfig.h
  brion.h
  circuit.h
  compartmentReport.h
  compartmentReportPlugin.h
  enums.h
  mesh.h
  morphology.h
  morphologyPlugin.h
  morphologyPlugin.ipp
  pluginInitData.h
  pluginLibrary.h
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
  log.h
  uint128_t.h
  uri.h
)

set(BRION_HEADERS
  constants.h
  detail/hdf5Mutex.h
  detail/json.hpp
  detail/mesh.h
  detail/meshBinary.h
  detail/skipWhiteSpace.h
  detail/utils.h
  detail/utilsHDF5.h
  md5/md5.hh
  plugin/compartmentReportBinary.h
  plugin/compartmentReportCommon.h
  plugin/compartmentReportDummy.h
  plugin/compartmentReportHDF5.h
  plugin/compartmentReportHDF5Sonata.h
  plugin/compartmentReportLegacyHDF5.h
  plugin/morphologyMORPHIO.h
  plugin/spikeReportASCII.h
  plugin/spikeReportBinary.h
  plugin/spikeReportBluron.h
  plugin/spikeReportHDF5.h
  plugin/spikeReportHDF5Sonata.h
  plugin/spikeReportNEST.h
  plugin/utilsHDF5.h
  serializable.h
  threadPool.h
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
  md5/md5.cc
  plugin/compartmentReportBinary.cpp
  plugin/compartmentReportCommon.cpp
  plugin/compartmentReportDummy.cpp
  plugin/compartmentReportHDF5.cpp
  plugin/compartmentReportHDF5Sonata.cpp
  plugin/compartmentReportLegacyHDF5.cpp
  plugin/morphologyMORPHIO.cpp
  plugin/spikeReportASCII.cpp
  plugin/spikeReportBinary.cpp
  plugin/spikeReportBluron.cpp
  plugin/spikeReportHDF5.cpp
  plugin/spikeReportHDF5Sonata.cpp
  plugin/spikeReportNEST.cpp
  plugin/utilsHDF5.cpp
  serializable.cpp
  threadPool.cpp
  uint128_t.cpp
  uri.cpp
  version.cpp
)

# Create library target
add_library(Brion SHARED ${BRION_PUBLIC_HEADERS} ${BRION_HEADERS} ${BRION_SOURCES})

# Compile options (CMake/CompileOptions.cmake)
compile_options(Brion)

# target_compile_options(Brion PRIVATE -Werror -O3)# -fno-strict-aliasing)
if(MSVC)
  target_compile_definitions(Brion PUBLIC /wd4251) # missing dll-interface for H5::Exception
endif()

# Include directories
# Normal includes
target_include_directories(Brion PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR};${PROJECT_SOURCE_DIR}/brion/plugin;${CMAKE_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include;include/brion>")
target_include_directories(Brion SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

# Link libraries
target_link_libraries(Brion PUBLIC ${GLM_TARGET})
target_link_libraries(Brion PRIVATE HighFive)
target_link_libraries(Brion PRIVATE MorphIO::morphio)
target_link_libraries(Brion PRIVATE ${Boost_FILESYSTEM_LIBRARIES})
target_link_libraries(Brion PRIVATE ${Boost_REGEX_LIBRARIES})
target_link_libraries(Brion PRIVATE ${Boost_IOSTREAMS_LIBRARIES})
target_link_libraries(Brion PRIVATE ${CMAKE_THREADS_LIB_INIT})

if(UNIX AND NOT APPLE)
  target_link_libraries(Brion PRIVATE dl)
  target_link_libraries(Brion PRIVATE rt)
endif()

set_target_properties(Brion
  PROPERTIES
  VERSION ${BRION_VERSION_STRING}
  SOVERSION ${BRION_VERSION_MAJOR}.${BRION_VERSION_MINOR}
)

# Export target
install(TARGETS Brion
  EXPORT Brion-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(FILES ${CMAKE_BINARY_DIR}/include/brion/version.h
  DESTINATION include/brion)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/brion
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ${PROJECT_SOURCE_DIR}/brion
  DESTINATION include
  FILES_MATCHING PATTERN "*.ipp")
