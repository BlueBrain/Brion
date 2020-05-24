include(ExternalProject)

# Mandatory dependencis not found in the system will be built
# from submodules

# --------------------------------------------------------------------------------
# GLM
# --------------------------------------------------------------------------------
if(NOT glm_FOUND AND NOT TARGET glm)
    message(STATUS "Gathering GLM from submodules")
    option(GLM_QUIET "No CMake Message" ON)
    option(GLM_TEST_ENABLE "Build unit tests" OFF)
    option(GLM_TEST_ENABLE_CXX_14 "Enable C++ 14" ON)
    set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ON)
    set(CMAKE_INSTALL_LIBDIR lib)
    add_subdirectory(glm)
    # WAR for https://github.com/g-truc/glm/issues/854
    if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
      target_compile_options(glm INTERFACE -Wno-error=class-memaccess)
    endif()
endif()

# --------------------------------------------------------------------------------
# sonata (which also brings HighFive)
# --------------------------------------------------------------------------------
if(NOT sonata_FOUND AND NOT TARGET sonata::sonata_shared)
    message(STATUS "Gathering sonata from submodules")
    add_subdirectory(libsonata)

    set(LIB_EXTENSION so)
    if(APPLE)
        set(LIB_EXTENSION dylib)
    endif()

    # Create external project for linkage
    # Add the dependencies with a different target name
    # in order not to clash with the previously added subidrectory
    ExternalProject_Add(libsonataextBrion
        PREFIX "libsonata"
        SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/libsonata"
        BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/deps/libsonata/lib/libsonata.${LIB_EXTENSION}
        CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/deps/libsonata
            -DEXTLIB_FROM_SUBMODULES:BOOL=ON
    )

    # Gather includes and libs from installed external project
    SET(LIBSONATA_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/libsonata/include)
    SET(LIBSONATA_LIB_DIR ${CMAKE_BINARY_DIR}/deps/libsonata/lib)

    # Generate imported target
    add_library(sonata::sonata_shared SHARED IMPORTED GLOBAL)
    set_target_properties(sonata::sonata_shared PROPERTIES IMPORTED_LOCATION ${LIBSONATA_LIB_DIR}/libsonata.${LIB_EXTENSION})
    set_target_properties(sonata::sonata_shared PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${LIBSONATA_INCLUDE_DIR})

    # Stablish dependency chain so mydepWrapper is not attemtped to be
    # built before the external one
    add_dependencies(sonata::sonata_shared libsonataextBrion)
endif()

# --------------------------------------------------------------------------------
# HighFive (If sonata came from a package but HighFive was not found)
# --------------------------------------------------------------------------------
if(NOT HighFive_FOUND AND NOT TARGET HighFive)
    message(STATUS "Gathering HighFive from submodules")
    add_subdirectory(HighFive)
endif()

# --------------------------------------------------------------------------------
# MVDTool
# --------------------------------------------------------------------------------
if(NOT MVDTool_FOUND AND NOT TARGET MVDTool)
    message(STATUS "Gathering MVDTool from submodules")
    find_package(Boost 1.41 QUIET REQUIRED COMPONENTS system)
    find_package(HDF5 QUIET REQUIRED)

    add_library(MVDTool INTERFACE)

    target_include_directories(MVDTool SYSTEM INTERFACE
          $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/MVDTool/include>
          $<INSTALL_INTERFACE:include>)
    target_include_directories(MVDTool SYSTEM INTERFACE ${Boost_INCLUDE_DIR})
    target_link_libraries(MVDTool INTERFACE HighFive)
    target_compile_definitions(MVDTool INTERFACE -DH5_USE_BOOST)
endif()
