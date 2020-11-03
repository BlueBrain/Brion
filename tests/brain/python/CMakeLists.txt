# Copyright (c) 2016, EPFL/Blue Brain Project
#                     Juan Hernando <juan.hernando@epfl.ch>
#                     Nadir Roman Guerrero <nadir.romanguerrero@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#
# Change this number when adding tests to force a CMake run: 3

# If the brion python binding target was not built, tests cannot be built
if(NOT TARGET brion_python)
  return()
endif()

# Gather all python tests
file(GLOB_RECURSE PYTHON_TESTS ${CMAKE_CURRENT_SOURCE_DIR} *.py)

# If BBPTestData is not present, exclude tests that require it
if(NOT TARGET BBPTestData)
    list(REMOVE_ITEM PYTHON_TESTS circuit.py)
    list(REMOVE_ITEM PYTHON_TESTS compartmentReport.py)
    list(REMOVE_ITEM PYTHON_TESTS morphology.py)
    list(REMOVE_ITEM PYTHON_TESTS spikes.py)
    list(REMOVE_ITEM PYTHON_TESTS simulation.py)
    list(REMOVE_ITEM PYTHON_TESTS synapses.py)
endif()

set(PYTHON_TESTS_DIR ${CMAKE_BINARY_DIR}/tests/python)
set(PYTHON_TEST_TARGETS)
foreach(PYTHON_TEST ${PYTHON_TESTS})

    # Geth the file name (strip out the path)
    get_filename_component(BASENAME ${PYTHON_TEST} NAME)
    # Set the path to the real test
    set(TEST_FILE ${PYTHON_TESTS_DIR}/${BASENAME})
    # Remove the .py
    string(REGEX REPLACE "\\.py$" "" BASENAME "${BASENAME}")
    # Generate a test name in the form python-test-{filename without .py}
    set(NAME python-test-${BASENAME})

    # Copy the file to the test directory
    configure_file(${PYTHON_TEST} ${TEST_FILE} COPYONLY)
    add_custom_command(OUTPUT ${TEST_FILE}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PYTHON_TEST} ${TEST_FILE}
      DEPENDS ${PYTHON_TEST})

    # Add the test
    add_test(NAME ${NAME} COMMAND ${PYTHON_EXECUTABLE} ${TEST_FILE})

    # Generate the command to run the test
    add_custom_target(ctest_${NAME}
      COMMAND ${CMAKE_CTEST_COMMAND} -T test --no-compress-output
        -R '^${NAME}$$' -C $<CONFIGURATION> \${ARGS}
      DEPENDS brion_python ${TEST_FILE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Running ${NAME} python test")
    set_target_properties(ctest_${NAME} PROPERTIES
      EXCLUDE_FROM_DEFAULT_BUILD ON FOLDER "Tests")

    # Add the test target to the python test target list
    set(PYTHON_TEST_TARGETS ${PYTHON_TEST_TARGETS} ctest_${NAME})

endforeach()

# Generate the target that will drive all python tests
add_custom_target(Brion-pythontests)
set_target_properties(Brion-pythontests PROPERTIES
  EXCLUDE_FROM_DEFAULT_BUILD ON FOLDER "tests")

# Make the global test target depends on this global python test target
add_dependencies(Brion-tests Brion-pythontests)

# Make th global python test target depends on all the python test targets
add_dependencies(Brion-pythontests ${PYTHON_TEST_TARGETS})

configure_file(setup.py.in ${PYTHON_TESTS_DIR}/setup.py)
