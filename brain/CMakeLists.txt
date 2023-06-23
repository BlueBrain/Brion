# Copyright (c) 2013-2017, EPFL/Blue Brain Project
#                          Juan Hernando <jhernando@fi.upm.es>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

# Generate the version include file
configure_file(version.h.in
               ${CMAKE_BINARY_DIR}/include/brain/version.h)

set(BRAIN_PUBLIC_HEADERS
  api.h
  brain.h
  circuit.h
  compartmentReport.h
  compartmentReportView.h
  compartmentReportMapping.h
  simulation.h
  neuron/morphology.h
  neuron/section.h
  neuron/soma.h
  neuron/types.h
  spikeReportReader.h
  spikeReportWriter.h
  synapse.h
  synapses.h
  synapsesIterator.h
  synapsesStream.h
  types.h
  log.h
  )

set(BRAIN_HEADERS
  detail/circuit.h
  detail/compartmentReport.h
  detail/synapsesStream.h
  neuron/morphologyImpl.h
  )

set(BRAIN_SOURCES
  circuit.cpp
  compartmentReport.cpp
  compartmentReportView.cpp
  compartmentReportMapping.cpp
  simulation.cpp
  neuron/morphology.cpp
  neuron/morphologyImpl.cpp
  neuron/section.cpp
  neuron/soma.cpp
  spikeReportReader.cpp
  spikeReportWriter.cpp
  synapse.cpp
  synapses.cpp
  synapsesIterator.cpp
  synapsesStream.cpp
  version.cpp
)

# Create library target
add_library(Brain SHARED ${BRAIN_PUBLIC_HEADERS} ${BRAIN_HEADERS} ${BRAIN_SOURCES})

# Compile options
#target_compile_options(Brain PRIVATE -Werror -O3)# -fno-strict-aliasing)
# Calls custom function to add compilation definitions (CMake/CompileOptions.cmake)
compile_options(Brain)

# Include directories
target_include_directories(Brain PUBLIC
                            "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR};${CMAKE_BINARY_DIR}/include>"
                            "$<INSTALL_INTERFACE:include>")
target_include_directories(Brain SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

# Link libraries
target_link_libraries(Brain PUBLIC Brion)
target_link_libraries(Brain PUBLIC ${GLM_TARGET})
target_link_libraries(Brain PRIVATE sonata::sonata_shared)
target_link_libraries(Brain PRIVATE HighFive)
target_link_libraries(Brain PRIVATE ${Boost_FILESYSTEM_LIBRARIES})
target_link_libraries(Brain PRIVATE MVDTool)
if(UNIX AND NOT APPLE)
    target_link_libraries(Brain PRIVATE dl)
    target_link_libraries(Brain PRIVATE rt)
endif()

set_target_properties(Brain
    PROPERTIES
        VERSION ${BRION_VERSION_STRING}
        SOVERSION ${BRION_VERSION_MAJOR}.${BRION_VERSION_MINOR}
)

# Export target
install(TARGETS Brain
    EXPORT Brion-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(FILES ${CMAKE_BINARY_DIR}/include/brain/version.h
        DESTINATION include/brain)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/brain
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

add_subdirectory(python)
