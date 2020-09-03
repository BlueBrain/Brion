# Copyright (c) 2013-2017, EPFL/Blue Brain Project
#                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
#                          Nadir Roman Guerrero <nadir.romanguerrero@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#
# Change this number when adding tests to force a CMake run: 3

# Generate the target that will drive all the tests

# Tests cannot be run without the test data
if(NOT BBPTestData_FOUND)
  message(STATUS "Skipping Brion Tests. Test generation requires test data")
  return()
endif()

# Generate the include file with paths to the test data used by the test
# executables
configure_file(paths.h.in ${PROJECT_BINARY_DIR}/tests/paths.h)

# Gather all cpp files
file(GLOB_RECURSE TEST_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR} *.cpp)

# Disabling SONATA tests based on the old standard, not valid anymore.
# Will be re-enabled when a test data with the new standard becomes available.
list(REMOVE_ITEM TEST_FILES brain/sonataCircuit.cpp)
list(REMOVE_ITEM TEST_FILES brain/sonataSimulation.cpp)

set(CPP_TEST_TARGETS)

# Generate a test for each file
foreach(FILE ${TEST_FILES})
    # Generate the target name from the file name
    string(REGEX REPLACE "\\.cpp$" "" NAME ${FILE})
    string(REGEX REPLACE "[./]" "-" NAME ${NAME})

    # Create test target
    add_executable(${NAME} ${FILE})

    # Add custom compile options
    compile_options(${NAME})

    # Generate main entry point for boost targets
    target_compile_definitions(${NAME} PUBLIC -DBOOST_TEST_DYN_LINK)

    # Place the test in the appropiate build folder
    set_target_properties(${NAME} PROPERTIES FOLDER tests OUTPUT_NAME ${NAME})

    # Include generated headers (paths)
    target_include_directories(${NAME} PRIVATE ${PROJECT_BINARY_DIR})

    # Link target with needed libraries
    target_link_libraries(${NAME} PRIVATE ${Boost_FILESYSTEM_LIBRARIES})
    target_link_libraries(${NAME} PRIVATE ${Boost_SYSTEM_LIBRARIES})
    target_link_libraries(${NAME} PRIVATE ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
    target_link_libraries(${NAME} PRIVATE BBPTestData)
    target_link_libraries(${NAME} PRIVATE Brion)
    target_link_libraries(${NAME} PRIVATE Brain)
    target_link_libraries(${NAME} PRIVATE HighFive)

    # Add the test
    add_test(NAME ${NAME}
      COMMAND $<TARGET_FILE:${NAME}>  -l test_suite
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    # Add the test target to the list of all cpp test targets
    set(CPP_TEST_TARGETS ${CPP_TEST_TARGETS} ${NAME})

    # Set test label to Brion unit testing
    set_tests_properties(${NAME} PROPERTIES LABELS "Brion-unit")

endforeach()

# Generate the target that will drive all cpp tests (make Brion-cpptests)
add_custom_target(Brion-cpptests USES_TERMINAL
    COMMAND ${CMAKE_CTEST_COMMAND} -T test --no-compress-output
    --output-on-failure -L Brion-unit -C $<CONFIGURATION> \${ARGS}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    COMMENT "Running Brion unit tests")
set_target_properties(Brion-cpptests PROPERTIES
  EXCLUDE_FROM_DEFAULT_BUILD ON FOLDER "tests")

# Make the global test target depends on this global python test target
add_dependencies(Brion-tests Brion-cpptests)

# Make th global python test target depends on all the python test targets
add_dependencies(Brion-cpptests ${CPP_TEST_TARGETS})

add_subdirectory(brain/python)
