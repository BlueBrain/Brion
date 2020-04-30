option(GLM_QUIET "No CMake Message" ON)
option(GLM_TEST_ENABLE "Build unit tests" OFF)
option(GLM_TEST_ENABLE_CXX_14 "Enable C++ 14" ON)
set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ON)
set(CMAKE_INSTALL_LIBDIR lib)


cmake_policy(VERSION 3.2)
set(GLM_VERSION "0.9.9")
project(glm VERSION ${GLM_VERSION} LANGUAGES CXX)
enable_testing()
file(GLOB ROOT_SOURCE .glm/glm/*.cpp)
file(GLOB ROOT_INLINE .glm/glm/*.inl)
file(GLOB ROOT_HEADER .glm/glm/*.hpp)
file(GLOB ROOT_TEXT .glm/*.txt)
file(GLOB ROOT_MD .glm/*.md)
file(GLOB ROOT_NAT .glm/util/glm.natvis)

file(GLOB_RECURSE CORE_SOURCE .glm/glm/detail/*.cpp)
file(GLOB_RECURSE CORE_INLINE .glm/glm/detail/*.inl)
file(GLOB_RECURSE CORE_HEADER .glm/glm/detail/*.hpp)

file(GLOB_RECURSE EXT_SOURCE .glm/glm/ext/*.cpp)
file(GLOB_RECURSE EXT_INLINE .glm/glm/ext/*.inl)
file(GLOB_RECURSE EXT_HEADER .glm/glm/ext/*.hpp)

file(GLOB_RECURSE GTC_SOURCE .glm/glm/gtc/*.cpp)
file(GLOB_RECURSE GTC_INLINE .glm/glm/gtc/*.inl)
file(GLOB_RECURSE GTC_HEADER .glm/glm/gtc/*.hpp)

file(GLOB_RECURSE GTX_SOURCE .glm/glm/gtx/*.cpp)
file(GLOB_RECURSE GTX_INLINE .glm/glm/gtx/*.inl)
file(GLOB_RECURSE GTX_HEADER .glm/glm/gtx/*.hpp)

file(GLOB_RECURSE SIMD_SOURCE .glm/glm/simd/*.cpp)
file(GLOB_RECURSE SIMD_INLINE .glm/glm/simd/*.inl)
file(GLOB_RECURSE SIMD_HEADER .glm/glm/simd/*.h)

source_group("Text Files" FILES ${ROOT_TEXT} ${ROOT_MD})
source_group("Core Files" FILES ${CORE_SOURCE})
source_group("Core Files" FILES ${CORE_INLINE})
source_group("Core Files" FILES ${CORE_HEADER})
source_group("EXT Files" FILES ${EXT_SOURCE})
source_group("EXT Files" FILES ${EXT_INLINE})
source_group("EXT Files" FILES ${EXT_HEADER})
source_group("GTC Files" FILES ${GTC_SOURCE})
source_group("GTC Files" FILES ${GTC_INLINE})
source_group("GTC Files" FILES ${GTC_HEADER})
source_group("GTX Files" FILES ${GTX_SOURCE})
source_group("GTX Files" FILES ${GTX_INLINE})
source_group("GTX Files" FILES ${GTX_HEADER})
source_group("SIMD Files" FILES ${SIMD_SOURCE})
source_group("SIMD Files" FILES ${SIMD_INLINE})
source_group("SIMD Files" FILES ${SIMD_HEADER})

add_library(glm INTERFACE)
target_include_directories(glm INTERFACE 
                            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/glm>  
                            $<INSTALL_INTERFACE:include/glm>)

if(BUILD_STATIC_LIBS)
add_library(glm_static STATIC ${ROOT_TEXT} ${ROOT_MD} ${ROOT_NAT}
	${ROOT_SOURCE}    ${ROOT_INLINE}    ${ROOT_HEADER}
	${CORE_SOURCE}    ${CORE_INLINE}    ${CORE_HEADER}
	${EXT_SOURCE}     ${EXT_INLINE}     ${EXT_HEADER}
	${GTC_SOURCE}     ${GTC_INLINE}     ${GTC_HEADER}
	${GTX_SOURCE}     ${GTX_INLINE}     ${GTX_HEADER}
	${SIMD_SOURCE}    ${SIMD_INLINE}    ${SIMD_HEADER})
	target_link_libraries(glm_static PUBLIC glm)
	add_library(glm::glm_static ALIAS glm_static)
endif()

if(BUILD_SHARED_LIBS)
add_library(glm_shared SHARED ${ROOT_TEXT} ${ROOT_MD} ${ROOT_NAT}
	${ROOT_SOURCE}    ${ROOT_INLINE}    ${ROOT_HEADER}
	${CORE_SOURCE}    ${CORE_INLINE}    ${CORE_HEADER}
	${EXT_SOURCE}     ${EXT_INLINE}     ${EXT_HEADER}
	${GTC_SOURCE}     ${GTC_INLINE}     ${GTC_HEADER}
	${GTX_SOURCE}     ${GTX_INLINE}     ${GTX_HEADER}
	${SIMD_SOURCE}    ${SIMD_INLINE}    ${SIMD_HEADER})
	target_link_libraries(glm_shared PUBLIC glm)
	add_library(glm::glm_shared ALIAS glm_shared)
endif()

install(TARGETS glm
    EXPORT BrionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(EXPORT BrionTargets
    DESTINATION share/Brion/glm/CMake
)

# WAR for https://github.com/g-truc/glm/issues/854
if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
  target_compile_options(glm INTERFACE -Wno-error=class-memaccess)
endif()