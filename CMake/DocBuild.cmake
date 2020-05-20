# From https://vicrucann.github.io/tutorials/quick-cmake-doxygen

# check if Doxygen is installed
find_package(Doxygen)
if(Doxygen_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/CMake/Doxyfile.in)
    set(DOXYGEN_OUT ${PROJECT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( Brion_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
    message(STATUS "Generating documentation with Doxygen")
else(Doxygen_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif(Doxygen_FOUND)
