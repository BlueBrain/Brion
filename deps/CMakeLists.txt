include(ExternalProject)

# Mandatory dependencis not found in the system will be built
# from submodules

# --------------------------------------------------------------------------------
# GLM
# --------------------------------------------------------------------------------
if(NOT glm_FOUND AND NOT TARGET glm)
    message(STATUS "Gathering GLM from submodules")

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

	    add_compile_options(-Werror -Weverything)
	    add_compile_options(-Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-c++11-long-long -Wno-padded -Wno-gnu-anonymous-struct -Wno-nested-anon-types)
	    add_compile_options(-Wno-undefined-reinterpret-cast -Wno-sign-conversion -Wno-unused-variable -Wno-missing-prototypes -Wno-unreachable-code -Wno-missing-variable-declarations -Wno-sign-compare -Wno-global-constructors -Wno-unused-macros -Wno-format-nonliteral)

    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")

	    add_compile_options(-O2)
	    add_compile_options(-Wno-long-long)

    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")

	    add_compile_options(/W4 /WX)
	    add_compile_options(/wd4309 /wd4324 /wd4389 /wd4127 /wd4267 /wd4146 /wd4201 /wd4464 /wd4514 /wd4701 /wd4820 /wd4365)
	    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    endif()

    add_definitions(-DGLM_FORCE_CXX14)

    file(GLOB ROOT_SOURCE glm/glm/*.cpp)
    file(GLOB ROOT_INLINE glm/glm/*.inl)
    file(GLOB ROOT_HEADER glm/glm/*.hpp)
    file(GLOB ROOT_TEXT glm//*.txt)
    file(GLOB ROOT_MD glm/*.md)
    file(GLOB ROOT_NAT glm//util/glm.natvis)

    file(GLOB_RECURSE CORE_SOURCE glm/glm/detail/*.cpp)
    file(GLOB_RECURSE CORE_INLINE glm/glm/detail/*.inl)
    file(GLOB_RECURSE CORE_HEADER glm/glm/detail/*.hpp)

    file(GLOB_RECURSE EXT_SOURCE glm/glm/ext/*.cpp)
    file(GLOB_RECURSE EXT_INLINE glm/glm/ext/*.inl)
    file(GLOB_RECURSE EXT_HEADER glm/glm/ext/*.hpp)

    file(GLOB_RECURSE GTC_SOURCE glm/glm/gtc/*.cpp)
    file(GLOB_RECURSE GTC_INLINE glm/glm/gtc/*.inl)
    file(GLOB_RECURSE GTC_HEADER glm/glm/gtc/*.hpp)

    file(GLOB_RECURSE GTX_SOURCE glm/glm/gtx/*.cpp)
    file(GLOB_RECURSE GTX_INLINE glm/glm/gtx/*.inl)
    file(GLOB_RECURSE GTX_HEADER glm/glm/gtx/*.hpp)

    file(GLOB_RECURSE SIMD_SOURCE glm/glm/simd/*.cpp)
    file(GLOB_RECURSE SIMD_INLINE glm/glm/simd/*.inl)
    file(GLOB_RECURSE SIMD_HEADER glm/glm/simd/*.h)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

    add_library(glm_static STATIC ${ROOT_TEXT} ${ROOT_MD} ${ROOT_NAT}
	    ${ROOT_SOURCE}    ${ROOT_INLINE}    ${ROOT_HEADER}
	    ${CORE_SOURCE}    ${CORE_INLINE}    ${CORE_HEADER}
	    ${EXT_SOURCE}     ${EXT_INLINE}     ${EXT_HEADER}
	    ${GTC_SOURCE}     ${GTC_INLINE}     ${GTC_HEADER}
	    ${GTX_SOURCE}     ${GTX_INLINE}     ${GTX_HEADER}
	    ${SIMD_SOURCE}    ${SIMD_INLINE}    ${SIMD_HEADER})

    target_include_directories(glm_static PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/glm>
                            $<INSTALL_INTERFACE:include>)

    add_library(glm_shared SHARED ${ROOT_TEXT} ${ROOT_MD} ${ROOT_NAT}
	    ${ROOT_SOURCE}    ${ROOT_INLINE}    ${ROOT_HEADER}
	    ${CORE_SOURCE}    ${CORE_INLINE}    ${CORE_HEADER}
	    ${EXT_SOURCE}     ${EXT_INLINE}     ${EXT_HEADER}
	    ${GTC_SOURCE}     ${GTC_INLINE}     ${GTC_HEADER}
	    ${GTX_SOURCE}     ${GTX_INLINE}     ${GTX_HEADER}
	    ${SIMD_SOURCE}    ${SIMD_INLINE}    ${SIMD_HEADER})

    target_include_directories(glm_shared PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/glm>
                            $<INSTALL_INTERFACE:include>)

    install(DIRECTORY glm DESTINATION include)

    add_library(glm INTERFACE)
    target_include_directories(glm INTERFACE
	    $<INSTALL_INTERFACE:include>
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/glm>)

    if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
      target_compile_options(glm INTERFACE -Wno-error=class-memaccess)
    endif()

    # Export target
    install(TARGETS glm
        EXPORT Brion-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include
    )
endif()

# --------------------------------------------------------------------------------
# sonata (which also brings HighFive)
# --------------------------------------------------------------------------------
if(NOT sonata_FOUND AND NOT TARGET sonata::sonata_shared)
    message(STATUS "Gathering sonata from submodules")
    # Sonata links against gcov, but on Apple this library is not included
    # on the library path
    if(APPLE)
        add_library(gcov INTERFACE)
        # Export target
        install(TARGETS gcov EXPORT gcov-targets)
        install(EXPORT gcov-targets DESTINATION share/gcov/CMake)
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-instr-generate -fcoverage-mapping")
        set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fprofile-instr-generate -fcoverage-mapping")
    endif()
    add_subdirectory(libsonata)
    add_library(sonatainter INTERFACE)
    target_include_directories(sonatainter INTERFACE ${CMAKE_CURRENT_LIST_DIR}/libsonata/include)
    target_link_libraries(sonatainter INTERFACE sonata_shared)
    add_library(sonata::sonata_shared ALIAS sonatainter)
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
    # WAR for https://github.com/g-truc/glm/issues/854
