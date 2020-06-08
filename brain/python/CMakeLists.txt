# Copyright (c) 2013-2018, EPFL/Blue Brain Project
#                          ahmet.bilgili@epfl.ch
#                          juan.hernando@epfl.ch
#                          nadir.romanguerrero@epfl.ch
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

# Cannot compile python bindings if no boost python or python interpreter is found

message("Boost python3 library: ${Boost_PYTHON3_LIBRARY}")
message("Python executable: ${PYTHON_EXECUTABLE}")
message("Numpy: ${NUMPY_FOUND}")

if(NOT Boost_PYTHON3_LIBRARY OR
    NOT PYTHON_EXECUTABLE OR NOT NUMPY_FOUND)
    message(STATUS "Not configured brain.py: Boost.Python, Python or Numpy not found")
    return()
endif()

set(BRAINPYTHONHELPERS_PUBLIC_HEADERS
    arrayHelpers.h
    arrayHelpers.inc
    helpers.h
)

set(BRAINPYTHONHELPERS_SOURCES
    arrayHelpers.cpp
    helpers.cpp
)

set(PYTHON_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/python)

# Create the python type translator helpers library
add_library(BrainPythonHelpers SHARED ${BRAINPYTHONHELPERS_PUBLIC_HEADERS} ${BRAINPYTHONHELPERS_SOURCES})
target_include_directories(BrainPythonHelpers PUBLIC
                                $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
                                $<INSTALL_INTERFACE:include/brain>)
target_include_directories(BrainPythonHelpers PRIVATE ${PYTHON_INCLUDE_PATH})
target_link_libraries(BrainPythonHelpers PUBLIC glm)
target_link_libraries(BrainPythonHelpers PUBLIC ${Boost_PYTHON3_LIBRARY})
target_link_libraries(BrainPythonHelpers PUBLIC Brain)

set_target_properties(BrainPythonHelpers
    PROPERTIES
        VERSION ${BRION_VERSION_STRING}
        SOVERSION ${BRION_VERSION_MAJOR}.${BRION_VERSION_MINOR}
)

# Export target
install(TARGETS BrainPythonHelpers
    EXPORT Brion-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

set(BRAIN_PYTHON_SOURCES
    brain.cpp
    circuit.cpp
    simulation.cpp
    compartmentReport.cpp
    spikeReportReader.cpp
    spikeReportWriter.cpp
    synapses.cpp
    spikes.cpp
    submodules.cpp
    test.cpp
    neuron/module.cpp
    neuron/morphology.cpp
)

add_library(brain_python MODULE ${BRAIN_PYTHON_SOURCES})

# Adds compile options using the custom function from CMake/CompileOptions.cmake
compile_options(brain_python)

target_include_directories(brain_python PRIVATE ${PYTHON_INCLUDE_PATH})
target_include_directories(brain_python PUBLIC ${PYTHON_INCLUDE_DIRS})
target_include_directories(brain_python PUBLIC ${PYTHON_NUMPY_INCLUDE_DIR})


if(TARGET ${PROJECT_NAME}-docstrings)
    add_dependencies(brain_python Brain ${PROJECT_NAME}-docstrings)
    target_compile_definitions(brain_python PRIVATE BRAIN_HAS_DOCSTRINGS)
endif()

target_link_libraries(brain_python PRIVATE BrainPythonHelpers)
target_link_libraries(brain_python PRIVATE Brain)
target_link_libraries(brain_python PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(brain_python PRIVATE ${Boost_PYTHON3_LIBRARY})
if(BBPTestData_FOUND)
  target_link_libraries(brain_python PRIVATE BBPTestData)
endif()

set_target_properties(brain_python PROPERTIES
    OUTPUT_NAME "_brain" PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/brain)

configure_file(brain/__init__.py.in ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/brain/__init__.py)

macro(COPY_AND_INSTALL_PACKAGE_DIRECTORY DIRECTORY)
    file(COPY brain/${DIRECTORY}
      DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/brain)

    install(DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/brain/${DIRECTORY}
      DESTINATION ${PYTHON_LIBRARY_SUFFIX}/brain
      COMPONENT COMMON)
endmacro()

install(TARGETS brain_python
  LIBRARY DESTINATION ${PYTHON_LIBRARY_SUFFIX}/brain)
install(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/brain/__init__.py
        DESTINATION ${PYTHON_LIBRARY_SUFFIX}/brain)
copy_and_install_package_directory(neuron)
