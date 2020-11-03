# Copyright (c) 2013-2018, EPFL/Blue Brain Project
#                          ahmet.bilgili@epfl.ch
#                          juan.hernando@epfl.ch
#                          nadir.romanguerrero@epfl.ch
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

# Cannot compile python bindings if no boost python or python interpreter is found

if(NOT Boost_PYTHON3_LIBRARY OR
    NOT PYTHON_EXECUTABLE OR NOT NUMPY_FOUND)
    if(BRION_REQUIRE_PYTHON)
        message( FATAL_ERROR "Not configured brion.py: Boost.Python, Python or Numpy not found. Build set to force python" )
    else()
        message(STATUS "Not configured brion.py: Boost.Python, Python or Numpy not found")
    endif()
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
target_include_directories(BrainPythonHelpers PRIVATE ${PYTHON_NUMPY_INCLUDE_DIR})
target_link_libraries(BrainPythonHelpers PUBLIC glm)
target_link_libraries(BrainPythonHelpers PUBLIC ${Boost_PYTHON3_LIBRARY})
target_link_libraries(BrainPythonHelpers PUBLIC Brain)
target_link_libraries(BrainPythonHelpers PUBLIC ${PYTHON_LIBRARIES})
target_link_libraries(BrainPythonHelpers PRIVATE ${Boost_PYTHON3_LIBRARY})

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

set(BRION_PYTHON_SOURCES
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

add_library(brion_python MODULE ${BRION_PYTHON_SOURCES})

# Adds compile options using the custom function from CMake/CompileOptions.cmake
compile_options(brion_python)

target_include_directories(brion_python PRIVATE ${PYTHON_INCLUDE_PATH})
target_include_directories(brion_python PUBLIC ${PYTHON_INCLUDE_DIRS})
target_include_directories(brion_python PUBLIC ${PYTHON_NUMPY_INCLUDE_DIR})


if(TARGET ${PROJECT_NAME}-docstrings)
    add_dependencies(brion_python Brain ${PROJECT_NAME}-docstrings)
    target_compile_definitions(brion_python PRIVATE BRAIN_HAS_DOCSTRINGS)
endif()

target_link_libraries(brion_python PRIVATE BrainPythonHelpers)
target_link_libraries(brion_python PRIVATE Brain)
target_link_libraries(brion_python PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(brion_python PRIVATE ${Boost_PYTHON3_LIBRARY})
if(BBPTestData_FOUND)
  target_link_libraries(brion_python PRIVATE BBPTestData)
endif()

if(BRION_USE_CUSTOM_LIB_DIR)
    set_target_properties(brion_python PROPERTIES
        OUTPUT_NAME "_brion" PREFIX "")
else()
    set_target_properties(brion_python PROPERTIES
        OUTPUT_NAME "_brion" PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/brion)
endif()

configure_file(brion/__init__.py.in ${CMAKE_BINARY_DIR}/lib/brion/__init__.py)

macro(COPY_AND_INSTALL_PACKAGE_DIRECTORY DIRECTORY)
    file(COPY brion/${DIRECTORY}
      DESTINATION ${CMAKE_BINARY_DIR}/lib/brion)

    install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/brion/${DIRECTORY}
      DESTINATION ${PYTHON_LIBRARY_SUFFIX}/brion
      COMPONENT COMMON)
endmacro()

install(TARGETS brion_python
  LIBRARY DESTINATION ${PYTHON_LIBRARY_SUFFIX}/brion)
install(FILES ${CMAKE_BINARY_DIR}/lib/brion/__init__.py
        DESTINATION ${PYTHON_LIBRARY_SUFFIX}/brion)
copy_and_install_package_directory(neuron)
