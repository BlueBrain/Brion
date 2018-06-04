# Copyright (c) 2013-2018, EPFL/Blue Brain Project
#                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#

set(BRIONPLUGINS_HEADERS
  compartmentReportBinary.h
  compartmentReportCommon.h
  compartmentReportDummy.h
  compartmentReportHDF5.h
  compartmentReportLegacyHDF5.h
  compartmentReportMap.h
  morphologyHDF5.h
  morphologySWC.h
  spikeReportASCII.h
  spikeReportBinary.h
  spikeReportBluron.h
  spikeReportHDF5.h
  spikeReportNEST.h
  utilsHDF5.h
)

set(BRIONPLUGINS_SOURCES
  compartmentReportBinary.cpp
  compartmentReportCommon.cpp
  compartmentReportDummy.cpp
  compartmentReportHDF5.cpp
  compartmentReportLegacyHDF5.cpp
  compartmentReportMap.cpp
  morphologyHDF5.cpp
  morphologySWC.cpp
  spikeReportASCII.cpp
  spikeReportBinary.cpp
  spikeReportBluron.cpp
  spikeReportHDF5.cpp
  spikeReportNEST.cpp
  utilsHDF5.cpp
)

set(BRIONPLUGINS_LINK_LIBRARIES
  PRIVATE Lunchbox Keyv Servus vmmlib HighFive ${Boost_FILESYSTEM_LIBRARIES}
          ${Boost_REGEX_LIBRARIES} ${Boost_IOSTREAMS_LIBRARIES}
          ${CMAKE_THREADS_LIB_INIT}
)

if(BRION_USE_ZEROEQ AND TARGET ZeroEQ)
  list(APPEND BRIONPLUGINS_HEADERS morphologyZeroEQ.h)
  list(APPEND BRIONPLUGINS_SOURCES morphologyZeroEQ.cpp)
  list(APPEND BRIONPLUGINS_LINK_LIBRARIES ZeroEQ)
endif()

set(BRIONPLUGINS_OMIT_LIBRARY_HEADER ON)
set(BRIONPLUGINS_OMIT_VERSION_HEADERS ON)
common_library(BrionPlugins)
