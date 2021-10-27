# Brion

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4472528.svg)](https://doi.org/10.5281/zenodo.4472528)


Welcome to Brion, a C++ project for read and write access to Blue Brain data
structures, including BlueConfig/CircuitConfig, Circuit, CompartmentReport,
Mesh, Morphology, Synapse and Target files.

Brion can be retrieved by cloning the
[source code](https://github.com/BlueBrain/Brion.git).
The [latest API documentation]
(http://bluebrain.github.io/Brion-1.9/index.html) can be found on
[bluebrain.github.io](http://bluebrain.github.io).
Additional documentation exists for the [Python wrapping of Brain]
(python/index.html).

To keep track of the changes between releases check the [changelog](@ref Changelog).

## Features

Brion provides two libraries Brion and Brain. The former is a collection of file
readers and writers intended for low level access to the data model. The latter
is a set of higher level classes that wrap low level data objects with a
use-case oriented API.

A python package to access the library can also be built and installed with pip. 
The package is available in PyPi as well, under the name "brion". Please, note that
the python package **requires the user to have the python development package installed
on their system**.

### IO library

This is the core library provided by Brion. It includes classes for reading
and writing files which store the Blue Brain data model.

* Fast and low-overhead read access to:
  * Blue configs (brion::BlueConfig)
  * Circuit description (brion::Circuit)
  * H5 Synapses data (brion::SynapseSummary, brion::Synapse)
  * Target (brion::Target)
  * BBP binary meshes (brion::Mesh)
  * BBP H5 morphologies and SWC morphologies (brion::Morphology)
  * Compartment reports (brion::CompartmentReport)
  * Spike reports (brion::SpikeReport)
* Fast and low-overhead write access to:
  * Compartment reports (brion::CompartmentReport)
  * BBP binary meshes (brion::Mesh)
  * BBP H5 morphologies (brion::Morphology)
  * Spike reports (brion::SpikeReport)
* Basic [data types](@ref brion/types.h) to work with the loaded data using
  [Boost](http://www.boost.org/doc/libs),
  [GLM](https://github.com/g-truc/glm).
  
#### Disclaimer

Although Brion is capable of reading SONATA format nodes, edges and reports, the use is
experimental and not supported officially. To read the SONATA format, it is encouraged
the usage of libsonata (https://github.com/BlueBrain/libsonata)

### High level library

The higher level library is called Brain and it provides:

* brain::Circuit to facilitate loading information about cells, morphologies (in
  local and global circuit coordinates) and synapses.
* brain::neuron::Morphology with higher level functions to deal with
  morphologies.
* brain::Synapses and brain::Synapse for array and object access to synapses.

## Building

Brion is a cross-platform library, designed to run on any modern operating
system, including all Unix variants. Brion uses CMake to create a
platform-specific build environment. The following platforms and build
environments are tested:

* Linux: Ubuntu 16.04 or above, RHEL 6.8 (Makefile, Ninja)

Building from source is as simple as:

    git clone --recursive https://github.com/BlueBrain/Brion.git
    mkdir Brion/build
    cd Brion/build
    cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DEXTLIB_FROM_SUBMODULES=ON ..
    ninja

## Funding & Acknowledgment

The development of this software was supported by funding to the Blue Brain Project,
a research center of the École polytechnique fédérale de Lausanne (EPFL), from the
Swiss government’s ETH Board of the Swiss Federal Institutes of Technology.

This project has received funding from the European Union’s FP7-ICT programme
under Grant Agreement No. 604102 (Human Brain Project RUP).

This project has received funding from the European Union's Horizon 2020 Framework
Programme for Research and Innovation under the Specific Grant Agreement No. 720270
(Human Brain Project SGA1).

This project is based upon work supported by the King Abdullah University of Science
and Technology (KAUST) Office of Sponsored Research (OSR) under Award No. OSR-2017-CRG6-3438.

## License

Brion is licensed under the LGPL, unless noted otherwise, e.g., for external dependencies.
See file LICENSE.txt for the full license.

Copyright (c) 2008-2021 Blue Brain Project/EPFL

This library is free software; you can redistribute it and/or modify it under the terms of the
GNU Lesser General Public License version 3 as published by the Free Software Foundation.

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with this library;
if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
MA 02110-1301 USA

