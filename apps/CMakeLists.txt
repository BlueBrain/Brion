# Copyright (c) 2020, EPFL/Blue Brain Project
#                    Stefan Eilemann <stefan.eilemann@epfl.ch>
#                    Nadir Román Guerrero <nadir.romanguerrero@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

# Make executables go into the build/bin folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(spikeConverter spikeConverter.cpp)
target_link_libraries(spikeConverter PRIVATE Brion)
target_link_libraries(spikeConverter PRIVATE ${Boost_PROGRAM_OPTIONS_LIBRARY})
if(TARGET BBPTestData)
  target_link_libraries(spikeConverter PRIVATE BBPTestData)
endif()

install(TARGETS spikeConverter
    EXPORT spikeConverter-targets
    RUNTIME DESTINATION bin
)

add_executable(morphologyConverter morphologyConverter.cpp)
target_link_libraries(morphologyConverter PRIVATE Brion)
target_link_libraries(morphologyConverter PRIVATE HighFive)
target_link_libraries(morphologyConverter PRIVATE ${Boost_PROGRAM_OPTIONS_LIBRARY})

install(TARGETS morphologyConverter
    EXPORT morphologyConverter-targets
    RUNTIME DESTINATION bin
)

add_executable(compartmentConverter compartmentConverter.cpp)
target_link_libraries(compartmentConverter PRIVATE Brion)
target_link_libraries(compartmentConverter PRIVATE ${Boost_SYSTEM_LIBRARIES})
target_link_libraries(compartmentConverter PRIVATE ${Boost_PROGRAM_OPTIONS_LIBRARY})
target_link_libraries(compartmentConverter PRIVATE ${Boost_FILESYSTEM_LIBRARIES})
if(TARGET BBPTestData)
  target_link_libraries(compartmentConverter PRIVATE BBPTestData)
endif()

install(TARGETS compartmentConverter
    EXPORT compartmentConverter-targets
    RUNTIME DESTINATION bin
)
